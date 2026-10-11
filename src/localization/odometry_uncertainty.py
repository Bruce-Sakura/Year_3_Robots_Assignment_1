"""Week 2 differential-drive odometry with optional measurement uncertainty.

The input is *joint angle feedback* (/joint_states), not commanded /cmd_vel.
Ground truth must NEVER be passed to this estimator. Its initial pose is known
from the simulation spawn configuration (0, 0, 0).

Sensing & Odometry lecture, slides 29-33:
  dl = rL * delta_phi_L
  dr = rR * delta_phi_R
  ds = (dr + dl) / 2
  dtheta = (dr - dl) / wheel_separation
  x' = x + ds*cos(theta + dtheta/2)
  y' = y + ds*sin(theta + dtheta/2)
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def integrate_odometry(pose: Pose2D, delta_s: float, delta_theta: float) -> Pose2D:
    # Midpoint heading approximates the curved path over one wheel sample.
    mid = pose.theta + delta_theta / 2.0
    return Pose2D(
        pose.x + delta_s * math.cos(mid),
        pose.y + delta_s * math.sin(mid),
        wrap_angle(pose.theta + delta_theta),
    )


class NoisyWheelOdometry:
    """Two simultaneous dead-reckoning estimates: baseline and noisy.

    Distance and heading measurement errors are drawn once PER joint sample,
    not once per ROS controller callback. Noise std devs are thus calibrated
    to the joint state publishing frequency (normally 50 Hz).
    """

    def __init__(
        self,
        left_radius: float,
        right_radius: float,
        wheel_separation: float,
        distance_noise_std_m: float = 0.0002,
        heading_noise_std_rad: float = 0.001,
        seed: int = 42,
        start_pose: Pose2D = Pose2D(),
    ):
        if not all(math.isfinite(v) and v > 0 for v in
                   (left_radius, right_radius, wheel_separation)):
            raise ValueError("Wheel radii and separation must be > 0")
        if not all(math.isfinite(v) and v >= 0 for v in
                   (distance_noise_std_m, heading_noise_std_rad)):
            raise ValueError("Noise standard deviations must be >= 0")
        self.left_radius = left_radius
        self.right_radius = right_radius
        self.separation = wheel_separation
        self.distance_noise_std_m = distance_noise_std_m
        self.heading_noise_std_rad = heading_noise_std_rad
        self.rng = random.Random(seed)
        self.clean_pose = start_pose
        self.noisy_pose = start_pose
        self._previous: tuple[float, float] | None = None
        self.steps = 0

    def update(self, left_total_rad: float, right_total_rad: float) -> tuple[Pose2D, Pose2D]:
        """Pass continuous unwrapped wheel angles, return (noisy, clean) pose."""
        values = (left_total_rad, right_total_rad)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Wheel angles must be finite")
        if self._previous is None:
            self._previous = values
            return self.noisy_pose, self.clean_pose

        delta_left = left_total_rad - self._previous[0]
        delta_right = right_total_rad - self._previous[1]
        self._previous = values

        ds_left = delta_left * self.left_radius
        ds_right = delta_right * self.right_radius
        delta_s = (ds_left + ds_right) / 2.0
        delta_theta = (ds_right - ds_left) / self.separation
        self.clean_pose = integrate_odometry(self.clean_pose, delta_s, delta_theta)

        # For truly stationary wheels, don't integrate noise indefinitely.
        # This is a measurement-noise model, NOT ground-truth pose corruption.
        if abs(delta_left) + abs(delta_right) > 1e-9:
            es = self.rng.gauss(0.0, self.distance_noise_std_m)
            etheta = self.rng.gauss(0.0, self.heading_noise_std_rad)
        else:
            es = etheta = 0.0

        self.noisy_pose = integrate_odometry(
            self.noisy_pose, delta_s + es, delta_theta + etheta
        )
        self.steps += 1
        return self.noisy_pose, self.clean_pose
