from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# Project paths
PROJECT_ROOT = Path(__file__).resolve().parents[1]
RESULTS_DIR = PROJECT_ROOT / "results"

# Automatically select latest CSV
CSV_FILENAME = "localization_*.csv"


def load_csv():
    files = list(RESULTS_DIR.glob(CSV_FILENAME))

    if not files:
        raise FileNotFoundError("No localization CSV found!")

    csv_path = max(files, key=lambda p: p.stat().st_mtime)

    print(f"Loading: {csv_path}")

    df = pd.read_csv(csv_path)

    if df.empty:
        raise ValueError("CSV file is empty!")

    return df, csv_path


def main():
    df, csv_path = load_csv()

    # # Heading diagnostic
    # gt_heading = np.degrees(df.theta_gt_rad.to_numpy())
    # clean_heading = np.degrees(df.theta_clean_rad.to_numpy())
    #
    # heading_error = (
    #                         (gt_heading - clean_heading + 180) % 360
    #                 ) - 180
    #
    # clean_error = df.clean_error_m.to_numpy()
    # noisy_error = df.position_error_m.to_numpy()
    #
    # noise_difference = np.hypot(
    #     df.x_est_m - df.x_clean_m,
    #     df.y_est_m - df.y_clean_m
    # )
    #
    # print("Clean RMSE:", np.sqrt(np.mean(clean_error ** 2)), "m")
    # print("Noisy RMSE:", np.sqrt(np.mean(noisy_error ** 2)), "m")
    # print("Noisy vs Clean final:", noise_difference.iloc[-1], "m")
    # print("Final Ground Truth heading:", gt_heading[-1], "deg")
    # print("Final Clean heading:", clean_heading[-1], "deg")
    # print("Final heading difference:", heading_error[-1], "deg")

    # Find when forward motion begins
    # travel = np.hypot(
    #     df.x_clean_m - df.x_clean_m.iloc[0],
    #     df.y_clean_m - df.y_clean_m.iloc[0]
    # )
    #
    # indices = np.flatnonzero(travel > 0.01)
    #
    # def heading_difference(index):
    #     gt = df.theta_gt_rad.iloc[index]
    #     clean = df.theta_clean_rad.iloc[index]
    #
    #     return np.degrees(
    #         np.arctan2(
    #             np.sin(gt - clean),
    #             np.cos(gt - clean)
    #         )
    #     )
    #
    # if len(indices) > 0:
    #     start = indices[0]
    #
    #     print(
    #         "Heading difference near DRIVE start:",
    #         heading_difference(start),
    #         "degrees"
    #     )
    #
    # print(
    #     "Heading difference at finish:",
    #     heading_difference(-1),
    #     "degrees"
    # )

    clean = df.theta_clean_rad.to_numpy()
    truth = df.theta_gt_rad.to_numpy()

    heading_error = np.degrees(
        np.arctan2(
            np.sin(truth - clean),
            np.cos(truth - clean)
        )
    )

    for angle in [0, 15, 30, 45]:
        target = np.radians(angle)
        index = np.argmin(np.abs(clean - target))

        print(
            f"Clean heading ≈ {angle}°: "
            f"Ground Truth = {np.degrees(truth[index]):.4f}°, "
            f"Error = {heading_error[index]:.4f}°"
        )

    # ===== Trajectory Plot =====
    fig1, ax1 = plt.subplots(figsize=(8, 6))

    ax1.plot(
        df.x_gt_m, df.y_gt_m,
        label="Ground Truth",
        linewidth=2
    )

    ax1.plot(
        df.x_est_m, df.y_est_m,
        "--",
        label="Noisy Odometry"
    )

    ax1.plot(
        df.x_clean_m, df.y_clean_m,
        ":",
        label="Clean Odometry"
    )

    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_title("Robot Localization Trajectory")
    ax1.axis("equal")
    ax1.legend()
    ax1.grid(True)

    trajectory_file = csv_path.with_name(
        csv_path.stem + "_trajectory.png"
    )

    fig1.savefig(trajectory_file, dpi=300, bbox_inches="tight")

    # ===== Error Analysis =====
    error = df.position_error_m.to_numpy()

    mse = np.mean(error ** 2)
    rmse = np.sqrt(mse)

    print(f"Samples: {len(df)}")
    print(f"MSE: {mse:.8f} m²")
    print(f"RMSE: {rmse:.6f} m")
    print(f"Final Error: {error[-1]:.6f} m")

    fig2, ax2 = plt.subplots(figsize=(8, 4))

    ax2.plot(
        df.time_s - df.time_s.iloc[0],
        error,
        label="Position Error"
    )

    ax2.axhline(
        rmse,
        linestyle="--",
        label=f"RMSE = {rmse:.5f} m"
    )

    ax2.set_xlabel("Simulation Time (s)")
    ax2.set_ylabel("Error (m)")
    ax2.set_title("Localization Error Over Time")
    ax2.legend()
    ax2.grid(True)

    error_file = csv_path.with_name(
        csv_path.stem + "_error.png"
    )

    fig2.savefig(error_file, dpi=300, bbox_inches="tight")

    print("Trajectory saved:", trajectory_file)
    print("Error plot saved:", error_file)

    # Display both figures
    plt.show()


if __name__ == "__main__":
    main()