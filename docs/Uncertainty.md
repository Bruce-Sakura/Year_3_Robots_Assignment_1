# Wheel odometry uncertainty and localization evaluation

## Estimation inputs

`src/localization/odometry_uncertainty.py` integrates unwrapped left/right wheel angles received through `/joint_states`. The initial planar pose is `(0, 0, 0)`, matching the world spawn's x, y, and yaw. Wheel geometry is read from `config/robot.yaml`: two 0.025 m radii and 0.10 m separation.

Two estimates are maintained: a clean baseline and an estimate with injected measurement noise. Neither uses `/odom` or ground truth as a pose correction. Navigation separately uses `/odom` to plan waypoint motion.

## Pose integration

For consecutive wheel readings:

```text
dl = rL * (phiL_current - phiL_previous)
dr = rR * (phiR_current - phiR_previous)
ds = (dr + dl) / 2
dtheta = (dr - dl) / b
mid = theta + dtheta / 2
x_next = x + ds * cos(mid)
y_next = y + ds * sin(mid)
theta_next = wrap(theta + dtheta)
```

The first reading establishes a baseline without integrating displacement. Clean odometry uses the increments directly. Noisy odometry replaces them with `ds + distance_error` and `dtheta + heading_error` before applying the same midpoint method.

## Noise configuration

```yaml
odometry_noise:
  distance_std_m: 0.0002
  heading_std_rad: 0.001
  seed: 42
```

These are independent zero-mean Gaussian standard deviations per moving wheel sample. The generator uses the configured seed. Noise is sampled when the sum of absolute wheel-angle increments exceeds `1e-9`; stationary samples receive no injected noise. Sampling follows incoming joint messages, configured at 50 Hz, rather than the 100 Hz control callback.

The values define a simulation experiment and are not measured sensor characteristics. Set both standard deviations to zero for a baseline comparison, and restart from the same initial pose for an independent trial. Robot geometry remains in `robot.yaml`; noise remains in `experiment.yaml`.

## Independent ground truth and time matching

The SDF PosePublisher publishes `/model/round_robot/ground_truth`, bridged to `/ground_truth` as `PoseStamped`. `LocalizationMetrics` uses its simulation timestamp to bracket pending estimates between two consecutive truth poses. It interpolates x and y linearly and heading along the shortest wrapped angular difference.

Samples older than the available bracket are dropped. Future samples wait for the next bracket. Truth intervals longer than 0.10 s are rejected. Duplicate or decreasing truth timestamps are ignored. The pending estimate queue is capped at 1,000 entries when truth is unavailable.

Ground truth is evaluation input only. It is not passed to the wheel estimator or motion planner. Reset the simulation and restart the Python process together for a new trial so the known initial estimator pose remains valid.

## Output and metrics

`results/localization_*.csv` contains `time_s`, noisy pose (`x_est_m`, `y_est_m`, `theta_est_rad`), clean pose, ground-truth pose, `position_error_m`, `clean_error_m`, and `heading_error_rad`.

For N matched samples:

```text
position_error = hypot(x_est - x_gt, y_est - y_gt)
clean_error = hypot(x_clean - x_gt, y_clean - y_gt)
heading_error = wrap(theta_est - theta_gt)
MSE = sum(position_error^2) / N
RMSE = sqrt(MSE)
clean_RMSE = sqrt(sum(clean_error^2) / N)
heading_RMSE = sqrt(sum(heading_error^2) / N)
```

Position RMSE is in metres; MSE is in square metres. The summary reports heading RMSE in degrees. Final localization error compares the last matched estimate with truth. Final goal error compares the last matched truth position with the latest requested waypoint. These measure different quantities.

The current application requests `(1, 2)`, `(2, 1)`, and `(3, 3)` metres, so the latest goal is normally `(3, 3)`. If execution stops earlier, the latest requested waypoint is used. With zero matched samples, the logger reports the missing data and CSV path without computing RMSE.

`results/odometry_*.csv` separately records `/odom` poses. `scrips/plot_localization.py` selects the most recently modified localization CSV in `results` and writes trajectory and error plots beside it. It takes no CSV path argument. Run instructions and the recorded three-waypoint results are in the root README.

Clean estimates can differ from truth due to wheel/contact behaviour and kinematic approximation. A noisy trajectory can momentarily have smaller error when random increments cancel another error. One seed and one trajectory do not establish statistical performance across trials.
