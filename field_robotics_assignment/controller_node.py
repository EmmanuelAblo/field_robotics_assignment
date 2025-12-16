#!/usr/bin/env python3
# controller_node.py
# ROS2 Humble — pose-to-pose + circle tracking controller

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import math
import time

def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))

class PoseCircleController(Node):
    def __init__(self):
        super().__init__('pose_circle_controller')

        # Publishers & subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.circle_sub = self.create_subscription(Float32MultiArray, '/circle_cmd', self.circle_cb, 10)

        # Controller gains 
        self.k_rho = 0.5        # linear gain
        self.k_alpha = 1.0      # angular gain to goal
        self.k_beta = -0.3    # angular gain final orientation

        # Velocity limits 
        self.v_max = 2.0       # m/s
        self.w_max = 2.5        # rad/s

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_ready = False

        # Target pose (2D)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.have_goal = False

        # Circle mode
        self.circle_active = False
        self.circle_center = (0.0, 0.0)
        self.circle_radius = 0.5
        self.circle_omega = 0.3   # rad/s
        self._circle_start_time = None
        self._circle_phase0 = 0.0

        # Control loop timer
        self.control_period = 0.05  # 20 Hz
        self.timer = self.create_timer(self.control_period, self.control_loop)

        self.get_logger().info("Pose & Circle controller started. Wait for /odom...")

    # --- callbacks ---
    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # quaternion to yaw
        q = msg.pose.pose.orientation
        # yaw extraction
        self.yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.odom_ready = True

    def goal_cb(self, msg: PoseStamped):
        # Accept goal from /goal_pose
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y

        # orientation yaw if provided (theta in quaternion)
        q = msg.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.target_yaw = yaw
        self.have_goal = True
        self.circle_active = False  # stop circle when a new pose is commanded
        self.get_logger().info(f"Received goal pose: x={self.target_x:.3f}, y={self.target_y:.3f}, yaw={self.target_yaw:.3f}")

    def circle_cb(self, msg: Float32MultiArray):
        # Expect: [center_x, center_y, radius, omega]
        data = msg.data
        if len(data) < 4:
            self.get_logger().error("circle_cmd expects 4 floats: [center_x, center_y, radius, omega]. Ignored.")
            return
        cx, cy, r, w = float(data[0]), float(data[1]), float(data[2]), float(data[3])
        if r <= 0 or abs(w) < 1e-6:
            # stop circle mode
            self.circle_active = False
            self.get_logger().info("Circle command invalid or radius<=0/omega~0 -> stopping circle mode.")
            return
        self.circle_center = (cx, cy)
        self.circle_radius = r
        self.circle_omega = w
        self._circle_start_time = self.get_clock().now().nanoseconds * 1e-9
        # compute initial phase so robot moves smoothly from current pose
        dx = self.x - cx
        dy = self.y - cy
        self._circle_phase0 = math.atan2(dy, dx)
        self.circle_active = True
        self.have_goal = True  # node will update target continuously from circle generator
        self.get_logger().info(f"Circle mode ON center=({cx:.2f},{cy:.2f}) r={r:.2f} w={w:.3f}")

    def control_loop(self):
        if not self.odom_ready:
            return

        # If circle active, update target from parametric circle
        if self.circle_active:
            t_now = self.get_clock().now().nanoseconds * 1e-9
            dt = t_now - (self._circle_start_time or t_now)
            theta = self._circle_phase0 + self.circle_omega * dt
            tx = self.circle_center[0] + self.circle_radius * math.cos(theta)
            ty = self.circle_center[1] + self.circle_radius * math.sin(theta)
            # desired yaw = tangent direction
            desired_yaw = theta + math.pi/2.0 if self.circle_omega > 0 else theta - math.pi/2.0
            self.target_x = tx
            self.target_y = ty
            self.target_yaw = normalize_angle(desired_yaw)
            self.have_goal = True

        if not self.have_goal:
            # no goal → publish zero velocities
            self.cmd_pub.publish(Twist())
            return

        # pose-to-pose controller
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        rho = math.hypot(dx, dy)           # distance to goal
        angle_to_goal = math.atan2(dy, dx)
        alpha = normalize_angle(angle_to_goal - self.yaw)
        beta = normalize_angle(self.target_yaw - angle_to_goal)

        # control law (classic)
        v = self.k_rho * rho
        w = self.k_alpha * alpha + self.k_beta * beta

        # if close to goal, slow down and consider goal reached
        if not self.circle_active:
            if rho < 0.03 and abs(normalize_angle(self.yaw - self.target_yaw)) < 0.05:
                # reached goal -> stop
                self.get_logger().info("Goal reached.")
                self.have_goal = False
                self.cmd_pub.publish(Twist())
                return

        # saturate
        v = max(min(v, self.v_max), -self.v_max)
        w = max(min(w, self.w_max), -self.w_max)

        # build Twist
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def set_target(self, x: float, y: float, yaw: float = 0.0):
        self.target_x = x
        self.target_y = y
        self.target_yaw = yaw
        self.have_goal = True
        self.circle_active = False
        self.get_logger().info(f"Programmatic target set: {x},{y},{yaw}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseCircleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
