# Wheel-angle waypoint navigation

## Implemented task

The root `main.py` executes `(1, 2)`, `(2, 1)`, and `(3, 3)` metres in sequence. Each waypoint has a 90-second timeout. `src/navigation.py` plans wheel displacements; `src/controller/robot_auto_controller.py` executes them using measured joints. `/odom` provides the starting pose for each plan.

## Differential-drive equations

The current radii are `rL = rR = 0.025 m`, and wheel separation is `b = 0.10 m`. These values agree between `config/robot.yaml` and the Gazebo DiffDrive plugin.

For wheel increments `dphiL` and `dphiR`:

```text
dl = rL * dphiL
dr = rR * dphiR
ds = (dr + dl) / 2
dtheta = (dr - dl) / b
```

The inverse equations give the wheel targets for a desired body displacement:

```text
dphiL = (ds - b*dtheta/2) / rL
dphiR = (ds + b*dtheta/2) / rR
```

## Planning and execution

For a starting pose `(x, y, theta)` and target `(target_x, target_y)`:

1. Compute the desired heading with `atan2(target_y-y, target_x-x)` and wrap its difference from the current heading to `[-pi, pi]`.
2. Plan an in-place turn with zero centre displacement.
3. Plan straight travel of `hypot(target_x-x, target_y-y)` with zero heading change.
4. Add a final in-place turn only when `go_to` receives a final heading.

The plan is fixed for the waypoint. Each segment starts from the current accumulated wheel angles and finishes when both wheels reach or pass their signed targets. This stopping rule checks wheel motion rather than distance from the target pose.

`WheelAngleTracker` unwraps successive joint-angle differences. It assumes each difference is smaller than pi radians. The first reading establishes the baseline. Scoped joint names are reduced to identify `left_wheel_joint` and `right_wheel_joint`.

The controller publishes `Twist` commands through Gazebo DiffDrive. The braking profile uses `sqrt(2*a*remaining)`, bounded by minimum and maximum speeds. A nonzero minimum allows target crossing. After each segment the controller sends zero velocity and waits 0.25 s for settling.

Both `/odom` and `/joint_states` must be fresh within one wall-clock second. Missing or stale feedback produces zero velocity. The control timer requests 100 Hz; SDF wheel feedback is configured at 50 Hz. On success, timeout, or interruption, `go_to` sends a stop command.

## Worked first-waypoint example

For the initial planar pose `(0, 0, 0)` and the first target `(1, 2)`:

```text
turn angle = atan2(2, 1) = 1.107149 rad = 63.434949 degrees
turn wheel targets = (-2.214297, +2.214297) rad
straight distance = sqrt(5) = 2.236068 m
straight wheel targets = (89.442719, 89.442719) rad
```

Later waypoints use the latest `/odom` pose rather than assuming exact arrival at the previous target.

## Logging and interpretation

`OdometryLogger` saves `/odom` poses. Independently, `NoisyWheelOdometry` maintains clean and noisy wheel estimates, and `LocalizationMetrics` compares them with timestamp-matched Gazebo truth. Ground truth never enters navigation or estimator correction.

Executing the wheel targets does not guarantee exact physical arrival: contact effects, slip, sampling, and residual movement can produce error. See the root README for running commands and `Uncertainty.md` for estimator and metric definitions.
