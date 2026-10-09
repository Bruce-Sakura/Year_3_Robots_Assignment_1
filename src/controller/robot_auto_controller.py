import math
import time

import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from controller.robot_controller import RobotController
from config.load_config import load_config
from navigation import calculate_motion
from src.localization.odometry_logger import OdometryLogger


class RobotAutoController(RobotController):
    def __init__(self):
        super().__init__()

        # Robot physical parameters
        config = load_config()
        self.left_wheel_radius = float(config["left_wheel_radius"])
        self.right_wheel_radius = float(config["right_wheel_radius"])
        self.wheel_separation = float(config["wheel_separation"])

        # Current estimated pose (x, y, theta)
        self.pose = None
        self.last_odom_time = None

        # Initialize position recorder
        self.position_recorder = OdometryLogger(self.get_logger())

        # Target pose
        self.target = None
        self.arrived = False

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.update_odometry,
            10
        )

        # Update control at 10 Hz
        self.timer = self.create_timer(
            0.1,
            self.control_step
        )

    def update_odometry(self, msg):
        """Update robot pose and record odometry."""

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # Quaternion -> Yaw
        theta = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y**2 + q.z**2)
        )

        # Update current estimated pose
        self.pose = (p.x, p.y, theta)
        self.last_odom_time = time.monotonic()

        # Record position to CSV and print logs
        self.position_recorder.record(msg)

    def control_step(self):
        """Calculate and publish velocity commands."""

        if self.target is None or self.arrived:
            return

        # Do not move without recent odometry
        if (
            self.pose is None
            or self.last_odom_time is None
            or time.monotonic() - self.last_odom_time > 1.0
        ):
            self.publisher.publish(Twist())
            return

        linear, angular, left, right, arrived = calculate_motion(
            *self.pose,
            *self.target,
            self.left_wheel_radius,
            self.right_wheel_radius,
            self.wheel_separation
        )

        if arrived:
            self.arrived = True
            self.target = None
            self.stop()
            self.get_logger().info("Target reached!")
            return

        # Publish body velocity
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        self.publisher.publish(msg)

    def go_to(self, x, y, theta=None, timeout=60.0):
        """Move to the target pose using odometry feedback."""

        self.target = (
            float(x),
            float(y),
            None if theta is None else float(theta)
        )
        self.arrived = False

        start_time = time.monotonic()

        self.get_logger().info(
            f"Navigating to ({x}, {y}, {theta})"
        )

        try:
            while rclpy.ok() and not self.arrived:
                if time.monotonic() - start_time > timeout:
                    raise TimeoutError("Navigation timed out")

                rclpy.spin_once(self, timeout_sec=0.1)
        finally:
            self.target = None
            self.stop()

        return self.arrived

    def close_log(self):
        """Close odometry CSV file."""
        self.position_recorder.close()