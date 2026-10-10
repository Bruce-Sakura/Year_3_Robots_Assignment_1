# Week 2: Odometry Uncertainty, Ground Truth, Error Analysis, and RMSE

This package was prepared based on the structure of the `main` branch of the public GitHub repository `Bruce-Sakura/Year_3_Robots_Assignment_1` as of 2026-10-09. **It has not automatically modified the GitHub repository and has not been tested in the user's actual Gazebo environment.**

## Implementation Overview

1. Preserve the existing `TURN → DRIVE` sequence, using wheel-joint angle feedback to complete movements. Ground truth is not used for navigation.
2. Calculate `Δs_L = r_L Δφ_L` and `Δs_R = r_R Δφ_R` from consecutive `/joint_states` readings, then compute `Δs = (Δs_L + Δs_R) / 2` and `Δθ = (Δs_R − Δs_L) / L`.
3. Maintain two independent dead-reckoning trajectories: **clean** and **noisy**. These use `(Δs, Δθ)` and `(Δs + e_s, Δθ + e_θ)`, respectively, to update `(x, y, θ)` using the midpoint method introduced in the lecture slides.
4. Model noise as `e_s ~ N(0, distance_std_m²)` and `e_θ ~ N(0, heading_std_rad²)`. Sample noise once for each incoming wheel-joint state message when the wheels are moving. The default wheel-state publication rate is 50 Hz.
5. Add an **independent model ground-truth publisher** in Gazebo (`/model/round_robot/ground_truth`), bridged to ROS 2 as `/ground_truth`. Ground truth is used **only for evaluation**, not for `NoisyWheelOdometry` calculations, planning, or execution.
6. Align and interpolate measurements using simulation message timestamps, record `position_error_m` for each frame, and calculate MSE, RMSE, final localization error, and the true final-position error relative to the target `(1, 1)`. Generate trajectory and error-versus-time plots.
7. Fix the duplicated DiffDrive plugin declarations in the repository's `model.sdf` (previously two declarations; only one should remain).

**Verified:** Python syntax; seven mathematical unit tests; first-time and repeated application of the installation script to a mock repository; shell syntax; SDF XML parsing; and CSV, MSE, RMSE, and Matplotlib plot generation using 481 simulated samples.

## Installation and Usage

Download `Robot_Week2_Uncertainty_Evaluation.zip` to your computer and extract it, for example, to `~/Downloads/Robot_Week2_Uncertainty_Evaluation/`.

Before applying the changes, inspect your local modifications to avoid unintentionally overwriting existing work:

```bash
cd ~/Desktop/Robots
git status -sb
# Consider backing up or committing your existing work first.
```

Run the installation script:

```bash
python ~/Downloads/Robot_Week2_Uncertainty_Evaluation/apply_to_repo.py ~/Desktop/Robots
```

The script automatically creates a timestamped `Robots_uncertainty_backup_.../` backup directory alongside the project directory. It does not create Git commits or push changes.

Run the tests from the project root:

```bash
cd ~/Desktop/Robots
python -m unittest discover -s tests -v
```

Restart Gazebo and the ROS 2 bridge:

```bash
bash start_sim.sh
```

In another terminal where the ROS 2 environment has been sourced, verify that ground-truth poses and wheel-joint feedback are being published:

```bash
ros2 topic echo /ground_truth --once
ros2 topic echo /joint_states --once
ros2 topic echo /odom --once
```

Then run the application from the repository root:

```bash
python main.py
```

When execution finishes, the terminal reports the total sample count, `MSE`, `RMSE`, `final-localization-error`, and `final true goal error`.

New experiment data is saved to `results/localization_YYYYMMDD_HHMMSS_microseconds.csv`. The existing `results/odometry_...csv` files continue to be generated as before.

To generate plots, ensure that `numpy`, `pandas`, and `matplotlib` are installed in your Conda environment:

```bash
CSV=$(ls -t results/localization_*.csv | head -1)
python scripts/plot_localization.py "$CSV"
```

This generates `..._trajectory.png` and `..._error.png`.

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
