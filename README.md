# Robotics Assignment 1

## Read the assignment articles

1. [Finding the heading error and deriving the effective wheel separation](docs/Differential_Drive_Heading_Error_Diagnosis_and_Effective_Wheel_Separation_EN.md) — CSV evidence, turn-phase diagnosis, reverse calculation to 0.08815039 m, and endpoint-error consistency.
2. [Correcting the wheel collision geometry and comparing recorded results](docs/Differential_Drive_Wheel_Contact_Diagnosis_and_Fix.md) — the 12 mm to 1 mm collision-width change, exact SDF edits, and before/after measurements.
3. [Wheel-angle waypoint navigation](docs/week2_turn_then_drive.md) — the implemented planner, feedback, and motion equations.
4. [Wheel odometry uncertainty and localization evaluation](docs/Uncertainty.md) — noise injection, independent truth, timestamp matching, and error metrics.

Recalculate the diagnostic evidence with `python scrips/analyze_heading.py`. The first two articles discuss archived single-goal experiments; the assignment results below describe the current three-waypoint run.


## Objective

This assignment studies two-dimensional localization of a differential-drive robot using noisy wheel odometry. The robot executes three world-frame waypoints in Gazebo: `(1, 2)`, `(2, 1)`, and `(3, 3)` metres. Clean and noisy wheel-based estimates are compared with an independently published simulator pose to measure localization error.

The implementation consists of robot simulation, turn-then-drive motion execution, wheel-based pose estimation, experiment logging, and trajectory/error plotting.

## Project files

```text
main.py                        Three-waypoint application
start_sim.sh                   Gazebo and ROS 2 bridge startup
config/
  robot.yaml                   Wheel geometry
  experiment.yaml              Noise settings and random seed
  load_config.py               Robot configuration loader
src/
  navigation.py                Wheel-motion equations and planning
  controller/
    robot_controller.py        Velocity publishing and timed motion helpers
    robot_auto_controller.py   Wheel-feedback execution and evaluation logging
  localization/
    odometry_logger.py         /odom CSV logging
    odometry_uncertainty.py     Clean and noisy wheel-based estimates
    localization_metrics.py    Timestamp matching and error metrics
scrips/                        Plotting and configuration repair helpers
Round_Robot/
  main.py                      Gazebo-only launcher
  build_robot.py               Blender visual-asset generator
  world.sdf                    Test world and robot spawn
  models/round_robot/           SDF model and visual meshes
  README.txt                   Model dimensions and asset notes
docs/                          Algorithm and recorded diagnosis notes
results/                       Current experiment CSV files and plots
  old/                         Archived experiment files
Report.docx                    Assignment report
```

The helper directory is named `scrips` in the repository. The Blender scene, preview images, and `parts.json` are stored in `Round_Robot`.

## Robot model and parameters

Values below come from `Round_Robot/models/round_robot/model.sdf`, `Round_Robot/world.sdf`, and `config/robot.yaml`.

| Parameter | Value |
|---|---:|
| Left and right wheel radius | 0.025 m |
| Wheel diameter | 0.050 m |
| Wheel centre-to-centre separation | 0.100 m |
| Left wheel centre in model coordinates | (0, 0.050, 0) m |
| Right wheel centre in model coordinates | (0, -0.050, 0) m |
| Wheel collision cylinder width | 0.001 m |
| Visual tyre width | 0.012 m |
| Base-link inertial mass | 0.220 kg |
| Each drive-wheel inertial mass | 0.025 kg |
| Each support-ball inertial mass | 0.004 kg |
| Support-ball radius | 0.005 m |
| Robot spawn pose in world | (0, 0, 0.025, 0, 0, 0) |
| Maximum physics step | 0.001 s |

The five model links are `base_link`, `left_wheel`, `right_wheel`, `front_ball`, and `rear_ball`. Decorative motors, brackets, wiring, and the concept board are visual parts. SDF masses and inertias are simulation parameters rather than measured hardware properties.

The planar initial estimator pose is `(x, y, theta) = (0, 0, 0)`; the world spawn height of 0.025 m is excluded from planar estimation. Positive X is forward, positive Y is left, and positive yaw is anticlockwise. The wheel axle midpoint is the `base_link` origin; the chassis geometric centre is 0.0125 m below it.

The same 0.025 m wheel radius is used by both wheel collision shapes, the DiffDrive plugin, and the Python geometry configuration. The commented 0.0881 m wheel separation is inactive. The active separation remains 0.10 m.

## Motion execution

`main.py` calls `go_to(x, y, timeout=90.0)` for each waypoint. The controller reads the latest `/odom` pose to plan an initial turn followed by straight travel. It computes fixed wheel-angle targets once per waypoint and executes them using `/joint_states`. Both wheels must reach or pass their signed targets before a segment finishes.

The controller slows near the target wheel displacement, sends zero velocity between segments, and waits 0.25 s for settling. It also stops when odometry or wheel feedback is older than one second. Its callback interval is 0.01 s, while configured wheel feedback is 50 Hz. Completion indicates that the planned wheel rotations were executed; wheel slip and overshoot can leave a true position error.

The current waypoint calls request no final heading. `go_to(x, y, theta)` supports an optional final turn. Detailed equations and feedback handling are documented in [Wheel-angle navigation](docs/week2_turn_then_drive.md).

For wheel angular speeds in rad/s:

```text
v = 0.0125 * (omega_R + omega_L)          # m/s
omega = 0.25 * (omega_R - omega_L)        # rad/s
omega_R = 40 * v + 2 * omega             # rad/s
omega_L = 40 * v - 2 * omega             # rad/s
```

## Localization and uncertainty

The estimator integrates unwrapped wheel-angle increments using the midpoint heading method. It maintains a clean baseline and a noisy estimate from the same initial pose. Gaussian errors are added to incremental centre displacement and heading only when the wheels move.

| Setting in `config/experiment.yaml` | Value |
|---|---:|
| Distance noise standard deviation per moving sample | 0.0002 m |
| Heading noise standard deviation per moving sample | 0.001 rad |
| Random seed | 42 |

Ground truth enters the evaluation logger only. It does not correct the wheel estimator or control the robot. The evaluator interpolates truth using simulation timestamps, rejecting intervals longer than 0.10 s. See [Odometry uncertainty and evaluation](docs/Uncertainty.md) for equations and metric definitions.

## Running the assignment

Use a Linux environment with ROS 2 Jazzy, Gazebo Harmonic, `ros_gz_bridge`, the required ROS message packages, and a Python interpreter that can import `rclpy` and `yaml`. Plotting additionally requires `numpy`, `pandas`, and `matplotlib`.

From the repository root, start Gazebo and the four topic bridges:

```bash
bash start_sim.sh
```

In a second terminal with the ROS environment loaded:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /odom --once
ros2 topic echo /joint_states --once
ros2 topic echo /ground_truth --once
python main.py
```

Run the experiment from a fresh robot spawn when comparing against the estimator's fixed initial pose. Restart the simulation before another independent trial; restarting Python alone does not reset the robot.

| ROS topic | Message type | Purpose |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Command Gazebo DiffDrive |
| `/odom` | `nav_msgs/msg/Odometry` | Navigation start pose and separate odometry log |
| `/joint_states` | `sensor_msgs/msg/JointState` | Wheel feedback and estimator input |
| `/ground_truth` | `geometry_msgs/msg/PoseStamped` | Independent evaluation pose |

The Gazebo topic names are `/model/round_robot/cmd_vel`, `/model/round_robot/odometry`, `/model/round_robot/joint_states`, and `/model/round_robot/ground_truth`. The startup script maps them to the ROS names above. Press Ctrl+C in the startup terminal to stop the bridge and simulation.

## Output and recorded results

`results/odometry_*.csv` stores `/odom` poses. `results/localization_*.csv` stores timestamp-matched clean/noisy estimates, ground truth, and errors. Generate plots with:

```bash
python scrips/plot_localization.py
```

The script selects the most recently modified localization CSV directly under `results`, accepts no CSV argument, and writes `_trajectory.png` and `_error.png` beside the selected CSV.

The following values were recalculated from [results/localization_20261010_105607_895741.csv](results/localization_20261010_105607_895741.csv), the dataset discussed in `Report.docx`:

| Metric | Recorded value |
|---|---:|
| Matched samples | 3,117 |
| Recorded time span | 62.32 s |
| Noisy position RMSE | 0.06022 m |
| Clean position RMSE | 0.02925 m |
| Noisy heading RMSE | 1.220 degrees |
| Final noisy localization error | 0.07502 m |
| Maximum noisy position error | 0.07671 m |
| Final noisy position | (3.020, 2.991) m |
| Final ground-truth position | (2.948, 3.011) m |

These are stored-run results, not a newly executed simulation. One seeded trial does not establish performance across different trials or noise levels.

## Troubleshooting

- Missing wheel feedback: inspect `/joint_states`, the JointStatePublisher plugin, and the bridge configuration. The controller stops on stale feedback.
- No matched evaluation samples: inspect `/ground_truth` and simulation timestamps. The logger reports missing matches rather than inventing RMSE.
- Model not found: check `Round_Robot/models/round_robot/model.sdf` and the `GZ_SIM_RESOURCE_PATH` set by `start_sim.sh`.
- Python cannot import ROS modules: use a compatible ROS-enabled interpreter. `scrips/fix-ros2.py` writes a Linux ROS package path into the active Python environment; it does not install ROS.
- Changed SDF or bridge settings: restart Gazebo and the bridge. `scrips/fix_diffdrive.py` and `scrips/fix_ground_truth.py` are repair helpers that edit configuration and create backups; inspect them before use.

## Assignment documents

- [Wheel-angle navigation](docs/week2_turn_then_drive.md): planning, execution, and equations.
- [Odometry uncertainty and evaluation](docs/Uncertainty.md): estimator, noise, time matching, and metrics.
- [Recorded heading diagnosis](docs/Differential_Drive_Heading_Error_Diagnosis_and_Effective_Wheel_Separation_EN.md): archived-run heading discrepancy and equivalent-separation calculation.
- [Wheel collision geometry diagnosis](docs/Differential_Drive_Wheel_Contact_Diagnosis_and_Fix.md): implemented collision-width change and archived-run comparison.
- [Model notes](Round_Robot/README.txt): visual dimensions and asset generation.
- `Report.docx`: assignment methods and results for the recorded three-waypoint run.
