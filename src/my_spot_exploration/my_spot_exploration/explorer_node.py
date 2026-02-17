import math
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

    def find_frontiers(self, grid: np.ndarray):
        frontiers = []
        rows, cols = grid.shape

        # Avoid borders
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if grid[r, c] != self.frontier_free_value:
                    continue
                nb = grid[r - 1:r + 2, c - 1:c + 2]
                if (nb == self.frontier_unknown_value).any():
                    frontiers.append((r, c))
        return frontiers

    def pick_frontier(self, frontiers, robot_rc, info):
        rr, rc = robot_rc
        best = None
        best_d2 = float('inf')

        for (r, c) in frontiers:
            gx, gy = self.grid_to_world_center(r, c, info)

            if self._blacklist_key(gx, gy) in self.blacklist:
                continue

            # Pick nearest frontier in grid space (cheap + good enough)
            d2 = (r - rr) ** 2 + (c - rc) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best = (r, c)

        return best

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

        frontiers = self.find_frontiers(grid)
        if not frontiers:
            self.get_logger().info("No frontiers found. Exploration may be complete.")
            return

        chosen = self.pick_frontier(frontiers, robot_rc, info)
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
