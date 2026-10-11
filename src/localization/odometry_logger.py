import csv
import math
import time

from datetime import datetime
from pathlib import Path


class OdometryLogger:
    """Record /odom poses, distinct from the wheel-estimate evaluation CSV."""

    def __init__(self, logger):
        self.logger = logger

        # Project root directory
        root = Path(__file__).resolve().parents[2]

        # Results directory
        results_dir = root / "results"
        results_dir.mkdir(parents=True, exist_ok=True)

        # Create CSV file
        filename = f"odometry_{datetime.now():%Y%m%d_%H%M%S}.csv"

        self.file = open(
            results_dir / filename,
            "w",
            newline="",
            encoding="utf-8"
        )

        self.writer = csv.writer(self.file)

        self.writer.writerow([
            "time",
            "x",
            "y",
            "theta",
        ])

        self.last_print_time = 0.0

    def record(self, msg):
        """Record robot odometry position."""

        position = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # Quaternion -> Yaw
        theta = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y**2 + q.z**2)
        )

        # Simulation timestamp
        timestamp = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

        # Save data
        self.writer.writerow([
            timestamp,
            position.x,
            position.y,
            theta
        ])

        self.file.flush()

        # Print every second
        now = time.monotonic()

        if now - self.last_print_time >= 1.0:
            self.logger.info(
                f"Position: "
                f"X={position.x:.4f}, "
                f"Y={position.y:.4f}, "
                f"Theta={math.degrees(theta):.2f}°"
            )

            self.last_print_time = now

    def close(self):
        if not self.file.closed:
            self.file.close()