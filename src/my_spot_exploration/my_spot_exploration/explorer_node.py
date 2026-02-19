import math
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import tf2_ros
from tf2_ros import TransformException


class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Explorer Node Started")

        # ----------------------------
        # Tunables
        # ----------------------------
        self.global_frame = 'map'
        self.base_frame = 'body'

        self.timer_period_sec = 3.0
        self.goal_cooldown_sec = 5.0    # don't send goals too frequently
        self.min_goal_distance_m = 0.3   # ignore goals closer than this
        self.minimum_frontier_cluster_size_cells = 6
        self.maximum_frontier_goal_distance_m = 8.0
        self.frontier_cluster_size_weight = 1.5

        # How we decide "frontier": free (0) next to unknown (-1)
        self.frontier_unknown_value = -1
        self.frontier_free_value = 0

        # Blacklist resolution (world coords rounding)
        self.blacklist_round_m = 0.2  # 10 cm buckets

        # ----------------------------
        # State
        # ----------------------------
        self.map_msg = None
        self.goal_in_progress = False
        self.blacklist = set()

        self.last_goal_time = self.get_clock().now()
        self.last_goal_world = None  # (x, y) of the last sent goal

        # ----------------------------
        # ROS Interfaces
        # ----------------------------
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(self.timer_period_sec, self.explore)

    # ----------------------------
    # Helpers
    # ----------------------------
    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    def _blacklist_key(self, x: float, y: float):
        # round to 10 cm (or whatever blacklist_round_m is)
        r = self.blacklist_round_m
        return (round(x / r) * r, round(y / r) * r)

    def get_robot_xy_in_map(self):
        # "latest available" transform
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            return (tf.transform.translation.x, tf.transform.translation.y)
        except TransformException as e:
            self.get_logger().warning(
                f"TF lookup failed ({self.global_frame}->{self.base_frame}): {e}"
            )
            return None

    def world_to_grid(self, x, y, info):
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        return row, col

    def grid_to_world_center(self, row, col, info):
        # center of cell (helps reduce edge weirdness)
        x = (col + 0.5) * info.resolution + info.origin.position.x
        y = (row + 0.5) * info.resolution + info.origin.position.y
        return x, y

    def _is_inside_grid(self, row_index: int, column_index: int, grid: np.ndarray) -> bool:
        return 0 <= row_index < grid.shape[0] and 0 <= column_index < grid.shape[1]

    def _is_traversable_free_cell(self, row_index: int, column_index: int, grid: np.ndarray) -> bool:
        if not self._is_inside_grid(row_index, column_index, grid):
            return False
        return grid[row_index, column_index] == self.frontier_free_value

    def _generate_four_connected_neighbors(self, row_index: int, column_index: int):
        return [
            (row_index - 1, column_index),
            (row_index + 1, column_index),
            (row_index, column_index - 1),
            (row_index, column_index + 1),
        ]

    def _generate_eight_connected_neighbors(self, row_index: int, column_index: int):
        return [
            (row_index - 1, column_index - 1), (row_index - 1, column_index), (row_index - 1, column_index + 1),
            (row_index, column_index - 1),                                         (row_index, column_index + 1),
            (row_index + 1, column_index - 1), (row_index + 1, column_index), (row_index + 1, column_index + 1),
        ]

    def _is_frontier_cell(self, row_index: int, column_index: int, grid: np.ndarray) -> bool:
        if not self._is_traversable_free_cell(row_index, column_index, grid):
            return False

        for neighbor_row, neighbor_column in self._generate_eight_connected_neighbors(row_index, column_index):
            if self._is_inside_grid(neighbor_row, neighbor_column, grid):
                if grid[neighbor_row, neighbor_column] == self.frontier_unknown_value:
                    return True

        return False

    def _nearest_traversable_cell(self, row_index: int, column_index: int, grid: np.ndarray):
        if self._is_traversable_free_cell(row_index, column_index, grid):
            return row_index, column_index

        breadth_first_queue = deque([(row_index, column_index)])
        visited_cells = {(row_index, column_index)}

        while breadth_first_queue:
            current_row, current_column = breadth_first_queue.popleft()

            for neighbor_row, neighbor_column in self._generate_four_connected_neighbors(current_row, current_column):
                if not self._is_inside_grid(neighbor_row, neighbor_column, grid):
                    continue
                if (neighbor_row, neighbor_column) in visited_cells:
                    continue

                visited_cells.add((neighbor_row, neighbor_column))

                if self._is_traversable_free_cell(neighbor_row, neighbor_column, grid):
                    return neighbor_row, neighbor_column

                breadth_first_queue.append((neighbor_row, neighbor_column))

        return None

    def find_frontier_clusters_wavefront(self, grid: np.ndarray, robot_row_column):
        robot_row, robot_column = robot_row_column
        nearest_robot_cell = self._nearest_traversable_cell(robot_row, robot_column, grid)
        if nearest_robot_cell is None:
            self.get_logger().warning("Could not find a traversable cell near robot pose.")
            return []

        map_visited = np.zeros(grid.shape, dtype=bool)
        frontier_visited = np.zeros(grid.shape, dtype=bool)
        map_breadth_first_queue = deque([nearest_robot_cell])
        map_visited[nearest_robot_cell[0], nearest_robot_cell[1]] = True

        frontier_clusters = []

        while map_breadth_first_queue:
            current_row, current_column = map_breadth_first_queue.popleft()

            if self._is_frontier_cell(current_row, current_column, grid) and not frontier_visited[current_row, current_column]:
                frontier_cluster_cells = []
                frontier_breadth_first_queue = deque([(current_row, current_column)])
                frontier_visited[current_row, current_column] = True

                while frontier_breadth_first_queue:
                    frontier_row, frontier_column = frontier_breadth_first_queue.popleft()
                    frontier_cluster_cells.append((frontier_row, frontier_column))

                    for neighbor_row, neighbor_column in self._generate_eight_connected_neighbors(frontier_row, frontier_column):
                        if not self._is_inside_grid(neighbor_row, neighbor_column, grid):
                            continue
                        if frontier_visited[neighbor_row, neighbor_column]:
                            continue
                        if not self._is_frontier_cell(neighbor_row, neighbor_column, grid):
                            continue

                        frontier_visited[neighbor_row, neighbor_column] = True
                        frontier_breadth_first_queue.append((neighbor_row, neighbor_column))

                if len(frontier_cluster_cells) >= self.minimum_frontier_cluster_size_cells:
                    frontier_clusters.append(frontier_cluster_cells)

            for neighbor_row, neighbor_column in self._generate_four_connected_neighbors(current_row, current_column):
                if not self._is_inside_grid(neighbor_row, neighbor_column, grid):
                    continue
                if map_visited[neighbor_row, neighbor_column]:
                    continue
                if not self._is_traversable_free_cell(neighbor_row, neighbor_column, grid):
                    continue

                map_visited[neighbor_row, neighbor_column] = True
                map_breadth_first_queue.append((neighbor_row, neighbor_column))

        return frontier_clusters

    def _pick_representative_cell_from_cluster(self, frontier_cluster_cells, robot_row_column):
        robot_row, robot_column = robot_row_column
        best_frontier_cell = None
        shortest_distance_squared = float('inf')

        for frontier_row, frontier_column in frontier_cluster_cells:
            distance_squared = (frontier_row - robot_row) ** 2 + (frontier_column - robot_column) ** 2
            if distance_squared < shortest_distance_squared:
                shortest_distance_squared = distance_squared
                best_frontier_cell = (frontier_row, frontier_column)

        return best_frontier_cell

    def pick_frontier_from_clusters(self, frontier_clusters, robot_row_column, info, robot_xy):
        rx, ry = robot_xy

        best_frontier_choice = None
        best_frontier_score = float('inf')

        for frontier_cluster_cells in frontier_clusters:
            representative_cell = self._pick_representative_cell_from_cluster(frontier_cluster_cells, robot_row_column)
            if representative_cell is None:
                continue

            representative_row, representative_column = representative_cell
            goal_world_x, goal_world_y = self.grid_to_world_center(representative_row, representative_column, info)

            if self._blacklist_key(goal_world_x, goal_world_y) in self.blacklist:
                continue

            dist_m = math.hypot(goal_world_x - rx, goal_world_y - ry) 
            cluster_size = len(frontier_cluster_cells)
            score = dist_m - self.frontier_cluster_size_weight * math.sqrt(cluster_size)

            if score < best_frontier_score:
                best_frontier_score = score
                best_frontier_choice = representative_cell

        return best_frontier_choice

    def _cooldown_ok(self) -> bool:
        now = self.get_clock().now()
        dt = (now - self.last_goal_time).nanoseconds * 1e-9
        return dt >= self.goal_cooldown_sec

    # ----------------------------
    # Nav2 goal handling
    # ----------------------------
    def send_goal(self, x, y):
        if self.goal_in_progress:
            return
        if not self.nav_client.server_is_ready():
            self.get_logger().warning("Nav2 action server not ready yet.")
            return

        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(f"Sending goal: ({x:.2f}, {y:.2f})")
        self.goal_in_progress = True
        self.last_goal_time = self.get_clock().now()
        self.last_goal_world = (float(x), float(y))

        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected by Nav2.")
            # reject -> blacklist it
            if self.last_goal_world is not None:
                self.blacklist.add(self._blacklist_key(*self.last_goal_world))
            self.goal_in_progress = False
            return

        self.get_logger().info("Goal accepted.")
        res_fut = goal_handle.get_result_async()
        res_fut.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        try:
            result = future.result()
            status = result.status
            self.get_logger().info(f"Nav result status: {status}")

            # Nav2 result status:
            # 4 = SUCCEEDED, 5 = CANCELED, 6 = FAILED  (common)
            if status != 4:
                self.get_logger().warning("Navigation did not succeed; blacklisting this goal.")
                if self.last_goal_world is not None:
                    self.blacklist.add(self._blacklist_key(*self.last_goal_world))

        except Exception as e:
            self.get_logger().error(f"Result callback exception: {e}")
            if self.last_goal_world is not None:
                self.blacklist.add(self._blacklist_key(*self.last_goal_world))
        finally:
            self.goal_in_progress = False

    # ----------------------------
    # Main exploration loop
    # ----------------------------
    def explore(self):
        if self.map_msg is None:
            self.get_logger().warning("No map yet.")
            return
        if self.goal_in_progress:
            return
        if not self._cooldown_ok():
            return

        robot_xy = self.get_robot_xy_in_map()
        if robot_xy is None:
            return

        info = self.map_msg.info
        grid = np.array(self.map_msg.data, dtype=np.int16).reshape((info.height, info.width))

        robot_rc = self.world_to_grid(robot_xy[0], robot_xy[1], info)

        frontier_clusters = self.find_frontier_clusters_wavefront(grid, robot_rc)
        if not frontier_clusters:
            self.get_logger().info("No reachable frontier clusters found. Exploration may be complete.")
            return

        chosen = self.pick_frontier_from_clusters(frontier_clusters, robot_rc, info, robot_xy)
        if chosen is None:
            self.get_logger().warning("No selectable frontier (all blacklisted?).")
            return

        gx, gy = self.grid_to_world_center(chosen[0], chosen[1], info)

        # Filter: ignore goals too close (prevents micro-goal thrash)
        dx = gx - robot_xy[0]
        dy = gy - robot_xy[1]
        dist = math.hypot(dx, dy)

        if dist < self.min_goal_distance_m:
            self.get_logger().info(
                f"Frontier too close ({dist:.2f} m < {self.min_goal_distance_m:.2f} m), skipping."
            )
            # Don't permanently blacklist close ones too aggressively; but we can for a while.
            self.blacklist.add(self._blacklist_key(gx, gy))
            return

        if dist > self.maximum_frontier_goal_distance_m:
            self.get_logger().info(
                f"Frontier too far ({dist:.2f} m > {self.maximum_frontier_goal_distance_m:.2f} m), skipping."
            )
            self.blacklist.add(self._blacklist_key(gx, gy))
            return

        # IMPORTANT: do NOT blacklist before attempting.
        # Only blacklist on rejection/failure in callbacks.
        self.send_goal(gx, gy)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
