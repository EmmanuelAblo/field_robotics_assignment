#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
import math


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # Map subscription
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_cb, 10)
        self.map_data = None

        # Odometry (robot pose)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.pose_ready = False

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Exploration state
        self.goal_in_progress = False
        self.min_frontier_distance = 0.6  # meters

        self.get_logger().info("Frontier explorer started... waiting for map and Nav2")

        # Timer (slow on purpose, exploration is event-driven)
        self.timer = self.create_timer(3.0, self.explore_loop)

    # ---------------- Callbacks ----------------

    def map_cb(self, msg):
        self.map_data = msg

    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.pose_ready = True

    # ---------------- Main loop ----------------

    def explore_loop(self):
        if self.map_data is None or not self.pose_ready:
            return

        if self.goal_in_progress:
            return

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            return

        frontiers = self.detect_frontiers()
        if not frontiers:
            self.get_logger().info("No frontiers detected — exploration completed")
            return

        goal = self.select_frontier(frontiers)
        if goal is None:
            self.get_logger().info("No suitable frontier found")
            return

        self.send_goal(goal)

    # ---------------- Frontier detection ----------------

    def detect_frontiers(self):
        width = self.map_data.info.width
        height = self.map_data.info.height
        data = np.array(self.map_data.data).reshape((height, width))

        frontiers = []
        for y in range(height):
            for x in range(width):
                if data[y, x] == 0:
                    neighbors = data[max(y-1, 0):min(y+2, height),
                                     max(x-1, 0):min(x+2, width)]
                    if -1 in neighbors:
                        frontiers.append((x, y))
        return frontiers

    # ---------------- Frontier selection ----------------

    def select_frontier(self, frontiers):
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin

        best_goal = None
        best_dist = float('inf')

        for x, y in frontiers:
            wx = origin.position.x + (x + 0.5) * resolution
            wy = origin.position.y + (y + 0.5) * resolution

            dist = math.hypot(wx - self.robot_x, wy - self.robot_y)

            if dist < self.min_frontier_distance:
                continue

            if dist < best_dist:
                best_dist = dist
                best_goal = (wx, wy)

        return best_goal

    # ---------------- Navigation ----------------

    def send_goal(self, goal):
        wx, wy = goal

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = wx
        pose.pose.position.y = wy
        pose.pose.orientation.w = 1.0
        goal_msg.pose = pose

        self.goal_in_progress = True
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_cb)

        self.get_logger().info(f"Sent goal to frontier at x={wx:.2f}, y={wy:.2f}")

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2")
            self.goal_in_progress = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future):
        result = future.result().result
        self.goal_in_progress = False
        self.get_logger().info("Frontier goal reached")

# ---------------- Main ----------------

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
