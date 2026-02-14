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

# This creates a ROS2 node named explorer.
# If you do ros2 node list, you’ll see /explorer.
class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Explorer Node Started")

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # TF: map -> body
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.base_frame = 'body'
        self.global_frame = 'map'

        self.map_msg = None
        self.goal_in_progress = False

        # blacklist failed goals in world coords (rounded)
        self.blacklist = set()

        # run exploration loop
        self.timer = self.create_timer(2.0, self.explore)

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    def get_robot_xy_in_map(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            return x, y
        except TransformException as e:
            self.get_logger().warning(f"TF lookup failed ({self.global_frame}->{self.base_frame}): {e}")
            return None

    def world_to_grid(self, x, y, info):
        # grid col = (x - origin_x)/res, row = (y - origin_y)/res
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        return row, col

    def grid_to_world(self, row, col, info):
        x = col * info.resolution + info.origin.position.x
        y = row * info.resolution + info.origin.position.y
        return x, y

    def find_frontiers(self, grid: np.ndarray):
        # frontier = free cell adjacent to unknown
        frontiers = []
        rows, cols = grid.shape
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if grid[r, c] != 0:
                    continue
                nb = grid[r-1:r+2, c-1:c+2]
                if (nb == -1).any():
                    frontiers.append((r, c))
        return frontiers

    def pick_frontier(self, frontiers, robot_rc, info):
        rr, rc = robot_rc
        best = None
        best_d2 = 1e18

        for (r, c) in frontiers:
            # convert to world, use that for blacklist key
            gx, gy = self.grid_to_world(r, c, info)
            key = (round(gx, 2), round(gy, 2))
            if key in self.blacklist:
                continue

            d2 = (r - rr) ** 2 + (c - rc) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best = (r, c)

        return best

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
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected by Nav2.")
            self.goal_in_progress = False
            return
        self.get_logger().info("Goal accepted.")
        res_fut = goal_handle.get_result_async()
        res_fut.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        try:
            result = future.result()
            status = result.status
            # status codes: 4=SUCCEEDED typically, but don’t hardcode logic too much
            self.get_logger().info(f"Nav result status: {status}")
            if status != 4:
                # blacklist last goal approx (we don’t have it stored here; simplest is to blacklist next time by failure logic)
                self.get_logger().warning("Navigation did not succeed; will pick another frontier.")
        except Exception as e:
            self.get_logger().error(f"Result callback exception: {e}")
        finally:
            self.goal_in_progress = False

    def explore(self):
        if self.map_msg is None:
            self.get_logger().warning("No map yet.")
            return
        if self.goal_in_progress:
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

        gx, gy = self.grid_to_world(chosen[0], chosen[1], info)
        # blacklist the goal we are about to attempt so we don’t resend instantly if it fails fast
        self.blacklist.add((round(gx, 2), round(gy, 2)))

        self.send_goal(gx, gy)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
