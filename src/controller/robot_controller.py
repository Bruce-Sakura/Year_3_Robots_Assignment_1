
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RobotController(Node):
    """Publish body velocity commands; timed motions have no pose feedback."""
    def __init__(self):
        super().__init__("maze_robot_controller")

        self.publisher = self.create_publisher(
            Twist, "/cmd_vel", 10
        )

    def drive(
        self,
        linear: float = 0.0,
        angular: float = 0.0,
        duration: float = 1.0,
    ):
        """Publish linear (m/s) and angular (rad/s) speed for a wall-clock duration."""

        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        end_time = time.monotonic() + duration

        while time.monotonic() < end_time:
            self.publisher.publish(msg)
            time.sleep(0.01) # 100Hz

        self.stop()

    def forward(self, speed=0.05, duration=1.0):
        self.drive(linear=speed, duration=duration)

    def backward(self, speed=0.05, duration=1.0):
        self.drive(linear=-speed, duration=duration)

    def turn_left(self, speed=1.0, duration=1.0):
        self.drive(angular=speed, duration=duration)

    def turn_right(self, speed=1.0, duration=1.0):
        self.drive(angular=-speed, duration=duration)

    def stop(self):
        msg = Twist()
        # Repeat zero velocity to improve delivery before shutdown.
        for _ in range(3):
            self.publisher.publish(msg)
            time.sleep(0.1)
