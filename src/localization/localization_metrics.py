"""Compare odometry with independently published Gazebo ground truth.

Timestamp synchronization is by simulation clock, interpolating the ground
truth pose between consecutive samples. MSE uses squared 2D Euclidean error;
RMSE is sqrt(MSE). The real-world pose is evaluation-only.
"""
from __future__ import annotations

import csv
import math
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from src.localization.odometry_uncertainty import Pose2D, wrap_angle


@dataclass(frozen=True)
class _Estimate:
    timestamp: float
    noisy: Pose2D
    clean: Pose2D


@dataclass(frozen=True)
class _Truth:
    timestamp: float
    pose: Pose2D


class LocalizationMetrics:
    def __init__(self, results_dir: Path, max_interpolation_gap_s: float = 0.10):
        results_dir.mkdir(parents=True, exist_ok=True)
        path = results_dir / f"localization_{datetime.now():%Y%m%d_%H%M%S_%f}.csv"
        self.path = path
        self._file = path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow([
            "time_s", "x_est_m", "y_est_m", "theta_est_rad",
            "x_clean_m", "y_clean_m", "theta_clean_rad",
            "x_gt_m", "y_gt_m", "theta_gt_rad",
            "position_error_m", "clean_error_m", "heading_error_rad",
        ])
        self._pending: deque[_Estimate] = deque()
        self._truth_before: _Truth | None = None
        self._truth_after: _Truth | None = None
        self.max_gap = max_interpolation_gap_s
        self.count = 0
        self.sum_squared_error = 0.0
        self.sum_squared_clean_error = 0.0
        self.sum_squared_heading_error = 0.0
        self.final_error: float | None = None
        self.final_clean_error: float | None = None
        self.final_ground_truth: Pose2D | None = None
        self.final_estimate: Pose2D | None = None
        self._goal: tuple[float, float] | None = None

    def set_goal(self, x: float, y: float) -> None:
        self._goal = (x, y)

    def add_estimate(self, stamp: float, noisy: Pose2D, clean: Pose2D) -> None:
        self._pending.append(_Estimate(stamp, noisy, clean))
        self._match()
        # Limit queue if ground-truth bridge is unavailable.
        while len(self._pending) > 1000:
            self._pending.popleft()

    def add_ground_truth(self, stamp: float, pose: Pose2D) -> None:
        new = _Truth(stamp, pose)
        if self._truth_after is not None:
            if stamp <= self._truth_after.timestamp:
                return  # Skip duplicates and simulation rewind.
            self._truth_before = self._truth_after
        self._truth_after = new
        self._match()

    def _match(self) -> None:
        before, after = self._truth_before, self._truth_after
        if before is None or after is None:
            return
        duration = after.timestamp - before.timestamp
        if duration <= 0:
            return
        while self._pending:
            sample = self._pending[0]
            if sample.timestamp < before.timestamp:
                self._pending.popleft()
                continue
            if sample.timestamp > after.timestamp:
                break  # Wait for next ground truth pair.
            self._pending.popleft()
            if duration > self.max_gap:
                continue  # Don't interpolate across a large simulation gap.
            fraction = (sample.timestamp - before.timestamp) / duration
            prev, curr = before.pose, after.pose
            gt = Pose2D(
                prev.x + fraction * (curr.x - prev.x),
                prev.y + fraction * (curr.y - prev.y),
                wrap_angle(prev.theta + fraction * wrap_angle(curr.theta - prev.theta)),
            )
            noisy, clean = sample.noisy, sample.clean
            error = math.hypot(noisy.x - gt.x, noisy.y - gt.y)
            clean_error = math.hypot(clean.x - gt.x, clean.y - gt.y)
            heading_error = wrap_angle(noisy.theta - gt.theta)
            self._writer.writerow([
                sample.timestamp, noisy.x, noisy.y, noisy.theta,
                clean.x, clean.y, clean.theta, gt.x, gt.y, gt.theta,
                error, clean_error, heading_error,
            ])
            self.count += 1
            self.sum_squared_error += error**2
            self.sum_squared_clean_error += clean_error**2
            self.sum_squared_heading_error += heading_error**2
            self.final_error = error
            self.final_clean_error = clean_error
            self.final_ground_truth = gt
            self.final_estimate = noisy
            if self.count % 25 == 0:
                self._file.flush()

    def summary(self) -> dict:
        if self.count == 0:
            return {"samples": 0, "csv": str(self.path)}
        mse = self.sum_squared_error / self.count
        result = {
            "samples": self.count,
            "mse_m2": mse,
            "rmse_m": math.sqrt(mse),
            "clean_rmse_m": math.sqrt(self.sum_squared_clean_error / self.count),
            "heading_rmse_deg": math.degrees(math.sqrt(self.sum_squared_heading_error / self.count)),
            "final_localization_error_m": self.final_error,
            "csv": str(self.path),
        }
        if self._goal is not None and self.final_ground_truth is not None:
            result["final_goal_error_m"] = math.hypot(
                self.final_ground_truth.x - self._goal[0],
                self.final_ground_truth.y - self._goal[1],
            )
        return result

    def close(self) -> dict:
        self._file.flush()
        self._file.close()
        return self.summary()
