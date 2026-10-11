# Week 2 differential-drive wheel-angle navigation

This is a **turn, then drive straight** demonstration for the current
`Round_Robot` Gazebo model. It is not SLAM, obstacle avoidance, or a
localization estimator.

## Current application

The root `main.py` runs `(1, 2)`, `(2, 1)`, and `(3, 3)` sequentially, with a 90-second timeout per waypoint. The `(1, 1)` calculation below is an illustrative single-goal example, not the current entry point. Each waypoint is planned from the latest `/odom` pose; wheel feedback executes the plan. Localization estimates and simulator truth are logged independently for evaluation.

## Physics and reference frames

The model uses `rL=rR=0.025 m` and wheel separation `b=0.10 m`, stored in
`config/robot.yaml`. Robot odometry is in `/odom`; measured Gazebo wheel joint
positions are in `/joint_states`. Both feeds must work before navigation starts.

The equations follow the Week 2 lecture (wheel distances -> body motion):

- `Δs = (ΔsL + ΔsR)/2`
- `Δθ = (ΔsR - ΔsL)/b`

Solving them for the wheel rotations gives:

- `ΔφL = (Δs - b·Δθ/2) / rL`
- `ΔφR = (Δs + b·Δθ/2) / rR`

The navigation plan is created **once** using the initial odometry pose:

1. `heading = atan2(goal_y - y, goal_x - x)`;
   `turn = wrap(heading - current_heading)`.
2. Turn in place: `Δs=0`, use `turn` to compute the signed wheel rotations.
3. Drive straight: `Δs=hypot(goal_x-x,goal_y-y)` and `Δθ=0`.
4. Optionally turn to a final heading *only* if `go_to(x, y, theta)` supplies one.

The robot actually drives via `Twist` and Gazebo's existing DiffDrive plugin.
Each segment ends when **both measured wheel joint rotations** reach or pass
those computed targets. There is no arbitrary 3 cm target-position tolerance.
Small speed limits and braking slow the wheels near their planned rotations.
This is **not** ideal position control: wheel slip, control period and braking
still produce real position error / wheel-angle overshoot.

For `(0,0,0) -> (1,1)` the first turn is 45° with planned wheel rotations
`(-π/2, +π/2)` rad. Forward wheel rotations are `sqrt(2)/0.025 ≈ 56.5685`
rad each (~9 turns).

## Running

1. Stop old Gazebo / bridge sessions. Run `bash start_sim.sh` from the repository root.
2. Confirm data in a second terminal: `ros2 topic echo /odom --once` and
   `ros2 topic echo /joint_states --once`.
3. In a ROS-enabled Python environment, run `python main.py`.
4. Inspect `results/odometry_*.csv` and the controller's logs for planned and
   measured wheel motion. The current `go_to(x, y)` calls do **not** adjust the final
   robot heading after the straight segment.

If `/joint_states` is missing, verify the JointStatePublisher plugin in
`Round_Robot/models/round_robot/model.sdf`, restart Gazebo and check the bridge.
The program fails safe by publishing zero velocity when position or joint
feedback is stale. Test only in an unobstructed environment: this algorithm
will not avoid walls.

## Assignment 1 notes

These measurements are used for motion execution and the existing logger records
Gazebo's published odometry. To assess localization you still need an **independent
noisy estimator** and Gazebo **ground truth**, which must not be used for navigation.
