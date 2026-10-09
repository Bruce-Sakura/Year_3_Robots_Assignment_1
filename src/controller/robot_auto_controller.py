"""Turn-then-drive controller using Gazebo wheel joint angle feedback.

The motion plan is computed once from the initial odometry pose. Gazebo
DiffDrive still actuates the wheels via /cmd_vel, while /joint_states tells us
when each planned wheel-angle target was actually reached or passed.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from config.load_config import load_config
from src.controller.robot_controller import RobotController
from src.localization.odometry_logger import OdometryLogger
from src.navigation import (
    WheelAngleTracker,
    motion_from_wheel_rotations,
    plan_turn_then_drive,
    wheel_speeds_from_twist,
    wheel_target_reached,
)


class RobotAutoController(RobotController):
    # Body speeds in m/s and rad/s; tune for your robot's physical response.
    MAX_LINEAR_SPEED = 0.12
    MIN_LINEAR_SPEED = 0.01
    LINEAR_DECELERATION = 0.15
    MAX_ANGULAR_SPEED = 0.50
    MIN_ANGULAR_SPEED = 0.05
    ANGULAR_DECELERATION = 0.80
    FEEDBACK_TIMEOUT = 1.0
    SETTLE_SECONDS = 0.25

    def __init__(self):
        super().__init__()
        config = load_config()
        self.left_wheel_radius = float(config["left_wheel_radius"])
        self.right_wheel_radius = float(config["right_wheel_radius"])
        self.wheel_separation = float(config["wheel_separation"])

        self.position_recorder = OdometryLogger(self.get_logger())
        self.pose = None
        self.last_odom_time = None
        self.wheel_angles = None
        self.last_joint_time = None
        self.wheel_tracker = WheelAngleTracker()

        self.target = None
        self.arrived = False
        self.plan = None
        self.phase = "IDLE"
        self.segments = []
        self.segment_index = 0
        self.segment_start = None
        self.segment = None
        self.settle_until = 0.0
        self.last_feedback_warning = 0.0
        self.last_motion_log = 0.0

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.update_odometry, 10
        )
        self.joints_sub = self.create_subscription(
            JointState, "/joint_states", self.update_joints, 10
        )
        self.timer = self.create_timer(0.05, self.control_step)

    def update_odometry(self, msg):
        """Use odometry only to establish the start pose and log the trajectory."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        theta = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z),
        )
        self.pose = (p.x, p.y, theta)
        self.last_odom_time = time.monotonic()
        self.position_recorder.record(msg)

    def update_joints(self, msg):
        """Read the two wheel joint angles, independently of the body pose."""
        names = [name.split("::")[-1].split("/")[-1] for name in msg.name]
        try:
            left_index = names.index("left_wheel_joint")
            right_index = names.index("right_wheel_joint")
            left = float(msg.position[left_index])
            right = float(msg.position[right_index])
            self.wheel_angles = self.wheel_tracker.update(left, right)
            self.last_joint_time = time.monotonic()
        except (ValueError, IndexError):
            # Ignore other robots' joint_states or malformed messages.
            pass

    def _fresh_feedback(self):
        now = time.monotonic()
        return (
            self.pose is not None
            and self.wheel_angles is not None
            and self.last_odom_time is not None
            and self.last_joint_time is not None
            and now - self.last_odom_time < self.FEEDBACK_TIMEOUT
            and now - self.last_joint_time < self.FEEDBACK_TIMEOUT
        )

    def _send_velocity(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publisher.publish(msg)

    def _next_segment(self):
        """Start next nonzero planned motion after the wheels have settled."""
        while self.segment_index < len(self.segments):
            segment = self.segments[self.segment_index]
            self.segment_index += 1
            # Ignore only floating-point zero-length segments, not goal tolerance.
            if abs(segment.left_rotation) < 1e-12 and abs(segment.right_rotation) < 1e-12:
                continue
            self.segment = segment
            self.segment_start = self.wheel_angles
            self.phase = segment.name
            self.get_logger().info(
                f"{segment.name}: centre={segment.distance:.4f} m, "
                f"heading={math.degrees(segment.heading_change):.3f} deg, "
                f"wheel angles L={segment.left_rotation:.5f}, "
                f"R={segment.right_rotation:.5f} rad"
            )
            return

        self.phase = "DONE"
        self.arrived = True
        self._send_velocity()
        self.get_logger().info(
            f"Motion completed. Final odometry pose={self.pose}; "
            "this is not Gazebo ground truth."
        )

    @staticmethod
    def _profile(remaining, minimum, maximum, deceleration):
        # Brake as measured wheel travel approaches the requested total.
        # The nonzero minimum prevents asymptotic stalling before target crossing.
        return max(minimum, min(maximum, math.sqrt(2.0 * deceleration * max(0.0, remaining))))

    def control_step(self):
        if self.target is None or self.arrived:
            return

        if not self._fresh_feedback():
            self._send_velocity()  # Fail safe: never drive on stale feedback.
            now = time.monotonic()
            if now - self.last_feedback_warning > 2.0:
                self.get_logger().warning(
                    "Waiting for /odom and /joint_states. "
                    "Restart start_sim.sh after updating the model and bridge."
                )
                self.last_feedback_warning = now
            return

        if self.plan is None:
            self.plan = plan_turn_then_drive(
                *self.pose,
                self.target[0], self.target[1],
                self.left_wheel_radius, self.right_wheel_radius,
                self.wheel_separation,
                target_theta=self.target[2],
            )
            self.segments = [self.plan.turn, self.plan.forward]
            if self.plan.final_turn is not None:
                self.segments.append(self.plan.final_turn)
            self.segment_index = 0
            self._next_segment()
            return

        if self.phase == "SETTLING":
            self._send_velocity()
            if time.monotonic() >= self.settle_until:
                self._next_segment()
            return

        if self.segment is None or self.segment_start is None:
            self._send_velocity()
            return

        left_delta = self.wheel_angles[0] - self.segment_start[0]
        right_delta = self.wheel_angles[1] - self.segment_start[1]
        segment = self.segment

        if (
            wheel_target_reached(left_delta, segment.left_rotation)
            and wheel_target_reached(right_delta, segment.right_rotation)
        ):
            self._send_velocity()
            self.get_logger().info(
                f"{segment.name} finished: measured L={left_delta:.5f}, "
                f"R={right_delta:.5f} rad"
            )
            self.phase = "SETTLING"
            self.settle_until = time.monotonic() + self.SETTLE_SECONDS
            return

        measured_distance, measured_heading = motion_from_wheel_rotations(
            left_delta, right_delta,
            self.left_wheel_radius, self.right_wheel_radius,
            self.wheel_separation,
        )

        if segment.name == "DRIVE":
            remaining = max(0.0, segment.distance - measured_distance)
            linear = self._profile(remaining, self.MIN_LINEAR_SPEED,
                                   self.MAX_LINEAR_SPEED, self.LINEAR_DECELERATION)
            angular = 0.0
        else:
            remaining = max(0.0, abs(segment.heading_change - measured_heading))
            linear = 0.0
            angular = math.copysign(
                self._profile(remaining, self.MIN_ANGULAR_SPEED,
                              self.MAX_ANGULAR_SPEED, self.ANGULAR_DECELERATION),
                segment.heading_change,
            )

        # Diagnosable wheel speed targets: DiffDrive converts Twist into these.
        left_speed, right_speed = wheel_speeds_from_twist(
            linear, angular,
            self.left_wheel_radius, self.right_wheel_radius,
            self.wheel_separation,
        )
        self._send_velocity(linear, angular)
        now = time.monotonic()
        if now - self.last_motion_log >= 1.0:
            self.get_logger().info(
                f"{segment.name}: v={linear:.3f} m/s, "
                f"omega={angular:.3f} rad/s; "
                f"wheel speeds L={left_speed:.3f}, R={right_speed:.3f} rad/s"
            )
            self.last_motion_log = now

    def go_to(self, x, y, theta=None, timeout=60.0):
        """Plan once, turn by measured wheel rotation, drive by measured rotation."""
        if self.target is not None:
            raise RuntimeError("Another go_to command is already active")
        self.target = (float(x), float(y), None if theta is None else float(theta))
        self.arrived = False
        self.plan = None
        self.phase = "WAITING"
        self.segment = None
        self.segment_start = None
        started = time.monotonic()
        self.get_logger().info(f"Navigating to {self.target}")

        try:
            while rclpy.ok() and not self.arrived:
                if time.monotonic() - started > timeout:
                    raise TimeoutError(
                        f"Turn-then-drive timed out in phase {self.phase}. "
                        "Check /joint_states and /odom."
                    )
                rclpy.spin_once(self, timeout_sec=0.1)
        finally:
            self.target = None
            self.phase = "IDLE"
            self._send_velocity()
        return self.arrived

    def close_log(self):
        self.position_recorder.close()
