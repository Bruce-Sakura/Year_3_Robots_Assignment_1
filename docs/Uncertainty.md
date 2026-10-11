# Week 2: Odometry Uncertainty, Ground Truth, Error Analysis, and RMSE

This document describes the implementation in this checkout. It separates wheel-based estimation, motion execution, and independent simulator-truth evaluation. Stored results are experiment evidence; no new Gazebo run was performed for this documentation update.

## Implementation Overview

1. Preserve the existing `TURN → DRIVE` sequence, using wheel-joint angle feedback to complete movements. Ground truth is not used for navigation.
2. Calculate `Δs_L = r_L Δφ_L` and `Δs_R = r_R Δφ_R` from consecutive `/joint_states` readings, then compute `Δs = (Δs_L + Δs_R) / 2` and `Δθ = (Δs_R − Δs_L) / L`.
3. Maintain two independent dead-reckoning trajectories: **clean** and **noisy**. These use `(Δs, Δθ)` and `(Δs + e_s, Δθ + e_θ)`, respectively, to update `(x, y, θ)` using the midpoint method introduced in the lecture slides.
4. Model noise as `e_s ~ N(0, distance_std_m²)` and `e_θ ~ N(0, heading_std_rad²)`. Sample noise once for each incoming wheel-joint state message when the wheels are moving. The default wheel-state publication rate is 50 Hz.
5. Add an **independent model ground-truth publisher** in Gazebo (`/model/round_robot/ground_truth`), bridged to ROS 2 as `/ground_truth`. Ground truth is used **only for evaluation**, not for `NoisyWheelOdometry` calculations, planning, or execution.
6. Align and interpolate measurements using simulation message timestamps, record `position_error_m` for each frame, and calculate MSE, RMSE, final localization error, and the true final-position error relative to the latest requested waypoint (currently the last waypoint is `(3, 3)`). Generate trajectory and error-versus-time plots.
7. Fix the duplicated DiffDrive plugin declarations in the repository's `model.sdf` (previously two declarations; only one should remain).

## Running the current implementation

The uncertainty modules are already integrated. No external installation ZIP or `apply_to_repo.py` is required, and this checkout does not contain a `tests` directory.

Use Linux with ROS 2 Jazzy, Gazebo Harmonic, `ros_gz_bridge`, and a Python environment that can import `rclpy`, `PyYAML`, `numpy`, `pandas`, and `matplotlib`.

From the repository root, start the simulation and bridges:

```bash
bash start_sim.sh
```

In a second ROS-enabled terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /ground_truth --once
ros2 topic echo /joint_states --once
ros2 topic echo /odom --once
python main.py
```

The application executes `(1, 2)`, `(2, 1)`, and `(3, 3)` metres sequentially. After execution it reports matched sample count, MSE, RMSE, clean RMSE, heading RMSE, final localization error, and true final goal error when available. With no matched truth samples, it reports the missing data rather than an RMSE.

New evaluation data is saved as `results/localization_YYYYMMDD_HHMMSS_microseconds.csv`. `/odom` poses are separately saved in `results/odometry_*.csv`. Historical runs remain in `results/old`.

Generate the two plots with:

```bash
python scrips/plot_localization.py
```

The script selects the most recently modified localization CSV in `results` and saves `_trajectory.png` and `_error.png` beside it. It currently takes no CSV argument.

For an existing model with missing topics, inspect `scrips/fix_diffdrive.py` and `scrips/fix_ground_truth.py` before applying them. These helpers edit simulation configuration and create backups. Restart Gazebo and the bridge after changing SDF or bridge settings.

## Error Metrics and Loss Definitions

- **Position error at time `t`:** `e_t = sqrt((x_est − x_gt)² + (y_est − y_gt)²)`.
- **Trajectory mean squared error (MSE):** `MSE = mean(e_t²)`, measured in `m²`.
- **Localization root mean squared error (RMSE):** `RMSE = sqrt(MSE)`, measured in `m`, providing a more interpretable reporting metric.
- **Final localization error:** `hypot(last_est_x − last_gt_x, last_est_y − last_gt_y)`.
- **Final goal error:** `hypot(last_gt_x − target_x, last_gt_y − target_y)`. This measures the true final position's distance from the target, **not** localization error.
- **Clean RMSE:** A baseline without additional random measurement noise, representing differences between the wheel-motion model and the ground-truth trajectory.

## Configuration

`config/experiment.yaml`:

```yaml
odometry_noise:
  distance_std_m: 0.0002   # Translation measurement standard deviation per 50-Hz wheel-state sample
  heading_std_rad: 0.001   # Heading measurement standard deviation per 50-Hz wheel-state sample
  seed: 42                 # Fixed random seed for reproducible experiments
```

For a baseline experiment, first set both standard deviations to `0.0`. Then increase each noise level separately to conduct comparative experiments. **This is a discrete measurement-noise model for experimentation. The parameter values are illustrative and are not based on measurements from physical sensors.** The resulting error magnitude depends on the JointState sampling rate.

Keep `config/robot.yaml` limited to the left and right wheel radii and wheel separation. Do not place the target position or experimental noise settings in this file.

## Timing and Update Rates

- `create_timer(0.01, ...)` requests a control update rate of **100 Hz**.
- `create_timer(0.0001, ...)` requests **10,000 Hz**, far above the current 50-Hz JointState update rate. This rate is not guaranteed to be achieved and does not necessarily improve accuracy.
- Gazebo's `max_step_size=0.001` specifies a maximum physics simulation time step of **0.001 s** (an idealized simulation step rate of 1 kHz), not the ROS 2 controller callback rate.
- The uncertainty model currently injects noise for each `/joint_states` message, independently of the controller's `create_timer()` frequency.

## Troubleshooting

- **No output from `/ground_truth`:** Make sure Gazebo has been **restarted** so that the modified SDF is reloaded. Check the Gazebo topics with `gz topic -l | grep ground_truth`.
- **A CSV is generated but no samples can be matched:** `LocalizationMetrics` reports `No timestamp-matched ground truth samples`. It **does not fabricate an RMSE value**.
- **Inconsistent timestamps or initial poses:** If topics use different time bases, or the robot's initial pose is not `(0, 0, 0)`, align the simulation timestamps and use the known initial pose before comparing trajectories.
- **Coursework evaluation:** To further meet the assignment requirements, consider repeated trials comparing different noise levels and random seeds.
- **Ground-truth separation:** During execution, the robot still uses `/odom` for initial-pose planning and `/joint_states` for control. Ground truth is never used for navigation.
