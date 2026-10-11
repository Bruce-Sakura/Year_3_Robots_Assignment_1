"""Two-stage differential-drive planning from Week 2 wheel-motion equations.

This module is ROS-independent: it plans wheel angular displacements, not
velocity commands. A controller must execute the plan using measured joints.
"""

import math
from dataclasses import dataclass


def normalize_angle(angle: float) -> float:
    """Return a signed heading difference in [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _check_geometry(left_radius: float, right_radius: float, separation: float) -> None:
    if any(not math.isfinite(v) or v <= 0 for v in (left_radius, right_radius, separation)):
        raise ValueError("Wheel radii and separation must be positive finite numbers")


@dataclass(frozen=True)
class WheelMotion:
    """One desired body displacement and its corresponding wheel rotations."""

    name: str
    distance: float          # Signed centre travel (m)
    heading_change: float    # Signed heading change (rad)
    left_rotation: float     # Left wheel rotation (rad)
    right_rotation: float    # Right wheel rotation (rad)


@dataclass(frozen=True)
class MotionPlan:
    """Turn toward goal, drive straight, optionally set final heading."""

    turn: WheelMotion
    forward: WheelMotion
    final_turn: WheelMotion | None
    target_x: float
    target_y: float


def wheel_rotations_from_motion(
    distance: float,
    heading_change: float,
    left_radius: float,
    right_radius: float,
    separation: float,
) -> tuple[float, float]:
    """Inverse of PPT Δs=(ΔsR+ΔsL)/2, Δθ=(ΔsR-ΔsL)/b."""
    _check_geometry(left_radius, right_radius, separation)
    # An in-place turn has opposite signed wheel travel; straight travel is shared.
    left_distance = distance - separation * heading_change / 2.0
    right_distance = distance + separation * heading_change / 2.0
    return left_distance / left_radius, right_distance / right_radius


def motion_from_wheel_rotations(
    left_rotation: float,
    right_rotation: float,
    left_radius: float,
    right_radius: float,
    separation: float,
) -> tuple[float, float]:
    """PPT forward kinematics: wheel angles -> distance and heading change."""
    _check_geometry(left_radius, right_radius, separation)
    left_distance = left_rotation * left_radius
    right_distance = right_rotation * right_radius
    return ((right_distance + left_distance) / 2.0,
            (right_distance - left_distance) / separation)


def wheel_speeds_from_twist(
    linear: float,
    angular: float,
    left_radius: float,
    right_radius: float,
    separation: float,
) -> tuple[float, float]:
    """Convert a body Twist to wheel angular velocity commands (rad/s)."""
    return wheel_rotations_from_motion(
        linear, angular, left_radius, right_radius, separation
    )


def _step(
    name: str,
    distance: float,
    heading_change: float,
    left_radius: float,
    right_radius: float,
    separation: float,
) -> WheelMotion:
    left, right = wheel_rotations_from_motion(
        distance, heading_change, left_radius, right_radius, separation
    )
    return WheelMotion(name, distance, heading_change, left, right)


def plan_turn_then_drive(
    x: float,
    y: float,
    theta: float,
    target_x: float,
    target_y: float,
    left_radius: float,
    right_radius: float,
    separation: float,
    target_theta: float | None = None,
) -> MotionPlan:
    """Calculate the two open-loop wheel-angle targets from the starting pose.

    The controller executes each segment with encoder feedback. This function
    does not implement obstacle avoidance or estimate the robot's pose.
    """
    _check_geometry(left_radius, right_radius, separation)
    values = (x, y, theta, target_x, target_y)
    if any(not math.isfinite(float(v)) for v in values):
        raise ValueError("Start and target coordinates must be finite")
    if target_theta is not None and not math.isfinite(target_theta):
        raise ValueError("Target heading must be finite")

    dx, dy = target_x - x, target_y - y
    distance = math.hypot(dx, dy)
    desired_heading = math.atan2(dy, dx) if distance > 0 else theta
    first_turn = normalize_angle(desired_heading - theta)

    turn = _step("TURN", 0.0, first_turn, left_radius, right_radius, separation)
    forward = _step("DRIVE", distance, 0.0, left_radius, right_radius, separation)

    final_turn = None
    if target_theta is not None:
        final_delta = normalize_angle(target_theta - desired_heading)
        final_turn = _step("FINAL_TURN", 0.0, final_delta,
                           left_radius, right_radius, separation)
    return MotionPlan(turn, forward, final_turn, target_x, target_y)


def wheel_target_reached(measured: float, target: float) -> bool:
    """Target passed in its signed direction; no position-distance tolerance."""
    return measured >= target if target >= 0 else measured <= target


class WheelAngleTracker:
    """Accumulate continuous wheel rotations even if a joint wraps at +-pi.

    Assumes successive joint-state samples differ by less than pi radians.
    """

    def __init__(self):
        self._previous: tuple[float, float] | None = None
        self.left = 0.0
        self.right = 0.0

    def update(self, left: float, right: float) -> tuple[float, float]:
        if not math.isfinite(left) or not math.isfinite(right):
            raise ValueError("Wheel angles must be finite")
        # The first sample establishes a baseline, not a displacement from zero.
        if self._previous is not None:
            self.left += normalize_angle(left - self._previous[0])
            self.right += normalize_angle(right - self._previous[1])
        self._previous = (left, right)
        return self.left, self.right
