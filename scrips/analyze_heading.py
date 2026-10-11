"""Recalculate the archived wheel-heading diagnosis without ROS dependencies.

Read the two recorded CSV files and print JSON evidence. This helper does not
change model parameters, overwrite results, or execute a new simulation.
"""
import csv
import json
import math
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def wrapped_degrees(angle):
    """Preserve the sign of an angular difference across the +/-pi boundary."""
    return math.degrees(math.atan2(math.sin(angle), math.cos(angle)))


def analyze(path):
    with path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"No samples in {path}")
    first, last = rows[0], rows[-1]
    clean_turn = wrapped_degrees(float(last["theta_clean_rad"]) - float(first["theta_clean_rad"]))
    true_turn = wrapped_degrees(float(last["theta_gt_rad"]) - float(first["theta_gt_rad"]))
    if abs(true_turn) < 1e-12:
        raise ValueError("An effective separation requires a nonzero true turn")
    checkpoints = []
    for angle in (0, 15, 30, 45):
        # Match by clean heading, as in plot_localization.py, not by true heading.
        sample = min(rows, key=lambda row: abs(float(row["theta_clean_rad"]) - math.radians(angle)))
        checkpoints.append({
            "requested_clean_heading_deg": angle,
            "time_s": float(sample["time_s"]),
            "clean_heading_deg": math.degrees(float(sample["theta_clean_rad"])),
            "truth_heading_deg": math.degrees(float(sample["theta_gt_rad"])),
            "truth_minus_clean_deg": wrapped_degrees(float(sample["theta_gt_rad"]) - float(sample["theta_clean_rad"])),
        })
    # This 1 cm threshold locates a diagnostic DRIVE checkpoint only.
    drive = next(row for row in rows if math.hypot(
        float(row["x_clean_m"]) - float(first["x_clean_m"]),
        float(row["y_clean_m"]) - float(first["y_clean_m"]),
    ) > 0.01)
    rmse = lambda key: math.sqrt(sum(float(row[key]) ** 2 for row in rows) / len(rows))
    final_difference = wrapped_degrees(float(last["theta_gt_rad"]) - float(last["theta_clean_rad"]))
    drive_difference = wrapped_degrees(float(drive["theta_gt_rad"]) - float(drive["theta_clean_rad"]))
    return {
        "csv": path.relative_to(PROJECT_ROOT).as_posix(),
        "samples": len(rows),
        "checkpoints": checkpoints,
        "drive_checkpoint_time_s": float(drive["time_s"]),
        "drive_heading_difference_deg": drive_difference,
        "final_heading_difference_deg": final_difference,
        "additional_drive_difference_deg": final_difference - drive_difference,
        "clean_turn_deg": clean_turn,
        "truth_turn_deg": true_turn,
        "effective_separation_m": 0.10 * clean_turn / true_turn,
        "rotation_scale": true_turn / clean_turn,
        "clean_rmse_m": rmse("clean_error_m"),
        "noisy_rmse_m": rmse("position_error_m"),
        "final_clean_error_m": float(last["clean_error_m"]),
        "final_noisy_error_m": float(last["position_error_m"]),
        "equal_distance_heading_error_prediction_m": 2 * math.sqrt(2) * math.sin(math.radians(abs(final_difference)) / 2),
    }


def main():
    """Analyze the specific archived before/after records used in the articles."""
    earlier = analyze(PROJECT_ROOT / "results/old/localization_20261009_230014_130403.csv")
    later = analyze(PROJECT_ROOT / "results/old/localization_20261010_102534_404782.csv")
    reductions = {
        key: 100 * (1 - later[key] / earlier[key])
        for key in ("final_heading_difference_deg", "clean_rmse_m", "noisy_rmse_m", "final_noisy_error_m")
    }
    print(json.dumps({"earlier": earlier, "later": later, "observed_reduction_percent": reductions}, indent=2))


if __name__ == "__main__":
    main()
