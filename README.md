# Maze Robot — ROS 2 & Gazebo Simulation

A differential-drive mobile robot simulation project developed using **ROS 2 Jazzy**, **Gazebo Harmonic**, **Python**, and **Blender**.

The project provides a modular framework for robot motion control, odometry, localization, sensor integration, and autonomous maze navigation.

The robot is modelled in Blender, exported as individual components, and assembled in Gazebo using the Simulation Description Format (SDF).

## 1. Project Overview

The primary objective is to develop and evaluate an intelligent mobile robot capable of navigating an initially unknown environment.

The project focuses on the following areas:

- Differential-drive robot modelling and control
- Robot simulation and physics
- ROS 2 communication
- Odometry and pose estimation
- Probabilistic localization
- Camera-based environment perception
- Autonomous mapping and navigation
- Maze exploration and obstacle avoidance
- Experimental evaluation and trajectory analysis

The system separates **simulation execution** from **robot control**, allowing Python algorithms to be developed, tested, and restarted without restarting Gazebo.

### Technology Stack

| Component | Technology |
|---|---|
| Operating System | Ubuntu 24.04 LTS |
| Robotics Framework | ROS 2 Jazzy |
| Simulation Engine | Gazebo Harmonic (Gazebo Sim 8) |
| Programming Language | Python 3.12 |
| ROS 2 Python Library | `rclpy` |
| ROS–Gazebo Communication | `ros_gz_bridge` |
| Robot Description | SDF 1.9 |
| Robot Modelling | Blender |
| 3D Mesh Formats | COLLADA (`.dae`) and STL (`.stl`) |
| Rendering Engine | OGRE |
| Version Control | Git |
| Development Environment | PyCharm / Linux Terminal |

**Rendering Compatibility:** The simulation uses the OGRE rendering engine to avoid OGRE2-related OpenGL crashes encountered with VMware virtual graphics.

---

## 2. Project Structure

```text
Robots/
│
├── models/
│   └── maze_robot/
│       ├── meshes/
│       │   ├── chassis.dae
│       │   ├── chassis.stl
│       │   ├── left_wheel.dae
│       │   ├── left_wheel.stl
│       │   ├── right_wheel.dae
│       │   ├── right_wheel.stl
│       │   ├── caster_ball.dae
│       │   ├── caster_ball.stl
│       │   ├── camera.dae
│       │   ├── camera.stl
│       │   ├── raspberry_pi.dae
│       │   └── raspberry_pi.stl
│       │
│       ├── model.config
│       ├── model.sdf
│       └── part_origins.json
│
├── results/
│
├── src/
│   ├── controller/
│   │   └── robot_controller.py
│   │
│   └── localization/
│
├── .gitignore
├── coordinate_frames.json
├── fix-ros2.py
├── main.py
├── README.md
├── robot_separate_objects.blend
├── start_sim.sh
└── world.sdf
```

### Directory Descriptions

#### `models/`

Contains robot models and their associated simulation resources.

The `maze_robot` model includes:

- Robot chassis
- Left and right drive wheels
- Rear caster ball
- Front camera
- Raspberry Pi representation
- Joint definitions
- Collision geometry
- Inertial properties
- Differential-drive plugin

#### `src/controller/`

Contains Python modules responsible for robot movement and control.

The `robot_controller.py` module provides the interface between Python programs and ROS 2 velocity commands.

#### `src/localization/`

Reserved for localization and pose estimation algorithms.

Planned implementations include differential-drive dead reckoning, noisy odometry integration, and probabilistic localization.

#### `results/`

Stores experiment outputs such as estimated trajectories, ground-truth trajectories, localization error plots, and simulation logs.

#### Main Project Files

| File | Description |
|---|---|
| `main.py` | Main entry point for Python robot control |
| `start_sim.sh` | Starts Gazebo and the ROS 2 bridge |
| `world.sdf` | Defines the simulation world |
| `model.sdf` | Defines the robot structure and physics |
| `model.config` | Gazebo model metadata |
| `coordinate_frames.json` | Robot coordinate reference definitions |
| `part_origins.json` | Model component origin information |
| `robot_separate_objects.blend` | Editable Blender robot model |
| `scrips/fix-ros2.py` | Local ROS 2 configuration or troubleshooting utility |
| `README.md` | Project documentation |

---

## 3. System Requirements

### Recommended Software

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3.12
- ROS 2 Python packages
- Gazebo ROS 2 integration packages
- Git
- PyCharm or another Python IDE

Blender is optional unless the robot's 3D geometry needs to be modified.

### Hardware Requirements

The computational requirements depend on the complexity of the simulation environment.

For a basic differential-drive robot and a small indoor maze, a modern laptop with an integrated GPU may be sufficient.

More demanding configurations, including high-resolution camera processing, complex environments, and multiple robots, may require stronger graphics and CPU performance.

Virtualized environments may have additional OpenGL compatibility limitations.

---

## 4. Installation

### 4.1 Install ROS 2 Jazzy

Follow the official ROS 2 installation guide:

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

After installation, initialize ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

Verify the ROS 2 environment:

```bash
echo $ROS_DISTRO
```

Expected output:

```text
jazzy
```

Verify that the ROS 2 CLI is available:

```bash
ros2 --help
```

### 4.2 Install Gazebo Harmonic

Install the ROS 2 and Gazebo integration packages:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz
```

Verify the Gazebo installation:

```bash
gz sim --version
```

Gazebo Harmonic uses Gazebo Sim 8.x.

### 4.3 Verify ROS 2 Bridge

Check whether the bridge package is installed:

```bash
ros2 pkg prefix ros_gz_bridge
```

If the package is unavailable, install it:

```bash
sudo apt install ros-jazzy-ros-gz-bridge
```

The bridge enables communication between ROS 2 nodes and Gazebo Transport.

### 4.4 Install Python Dependencies

The primary ROS 2 Python dependencies are:

- `rclpy`
- `geometry_msgs`
- `nav_msgs`

These are normally installed with the appropriate ROS 2 packages.

Verify Python compatibility:

```bash
/usr/bin/python3 --version
```

Verify the ROS 2 Python library:

```bash
/usr/bin/python3 -c "import rclpy; print('ROS 2 Python ready')"
```

For ROS 2 Jazzy on Ubuntu 24.04, the system Python interpreter is recommended.

---

## 5. Environment Configuration

### 5.1 Configure ROS 2 Automatically

Open the Bash configuration file:

```bash
nano ~/.bashrc
```

Add the following line if it is not already present:

```bash
source /opt/ros/jazzy/setup.bash
```

Apply the changes:

```bash
source ~/.bashrc
```

This makes ROS 2 available automatically in new interactive Bash terminals.

### 5.2 Configure Gazebo Model Resources

Gazebo must know where the custom robot models are stored.

For the current project location:

```text
/home/user/Desktop/Robots/models
```

Add this line to `~/.bashrc`:

```bash
export GZ_SIM_RESOURCE_PATH="$HOME/Desktop/Robots/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
```

Reload the configuration:

```bash
source ~/.bashrc
```

Verify the environment variable:

```bash
echo $GZ_SIM_RESOURCE_PATH
```

The path should include:

```text
/home/user/Desktop/Robots/models
```

The `start_sim.sh` script also configures the model search path automatically, making the permanent setting optional for normal project execution.

### 5.3 Configure PyCharm

Open the project directory:

```text
/home/user/Desktop/Robots
```

Use the following Python interpreter:

```text
/usr/bin/python3
```

Configure the working directory:

```text
/home/user/Desktop/Robots
```

Ensure ROS 2 environment variables are available when launching Python from PyCharm.

A ROS 2 Python program may fail when launched from an IDE that has not inherited the environment created by:

```bash
source /opt/ros/jazzy/setup.bash
```

One approach is to launch PyCharm from a terminal where ROS 2 has already been initialized.

Conda environments may conflict with the system-installed ROS 2 Python packages if they use a different Python version or incompatible dependencies.

---

## 6. Robot Model

The robot uses a two-wheel differential-drive configuration supported by a rear caster ball.

### 6.1 Robot Components

| Component | Description |
|---|---|
| `base_link` | Main robot body |
| `left_wheel` | Left driven wheel |
| `right_wheel` | Right driven wheel |
| `caster_ball` | Rear passive support |
| `camera` | Front-facing camera |
| `raspberry_pi` | Onboard computing representation |

Each component is defined as a separate link in the robot's SDF model.

### 6.2 Physical Parameters

| Parameter | Value |
|---|---|
| Drive Type | Differential drive |
| Wheel Radius | 0.02 m |
| Wheel Diameter | 0.04 m |
| Wheel Separation | 0.10 m |
| Chassis Mass | 0.70 kg |
| Left Wheel Mass | 0.0333 kg |
| Right Wheel Mass | 0.0333 kg |
| Forward Direction | Positive X-axis |
| Left Direction | Positive Y-axis |
| Upward Direction | Positive Z-axis |

The robot's physical properties are defined in:

```text
models/maze_robot/model.sdf
```

### 6.3 Coordinate Frames

The model uses a right-handed coordinate system:

- **X-axis:** Forward
- **Y-axis:** Left
- **Z-axis:** Upward

The SDF model defines three important frames.

**Model Frame**

```text
model_center
```

Located at the model origin:

```text
(0.000, 0.000, 0.000)
```

**Base Link Frame**

```text
base_link
```

Its location relative to the model frame is:

```text
(0.025, 0.000, 0.021)
```

**Axle Center Frame**

```text
axle_center
```

The axle center is attached to `base_link` with zero relative translation and rotation.

Therefore, the axle center currently coincides with the `base_link` origin.

### 6.4 Wheel Positions

The wheel centers are positioned relative to the model frame as follows:

```text
Left Wheel:
X =  0.025 m
Y =  0.050 m
Z =  0.021 m

Right Wheel:
X =  0.025 m
Y = -0.050 m
Z =  0.021 m
```

The wheel separation is:

$$
L = 0.050 - (-0.050) = 0.100 \text{ m}
$$

The axle center is the midpoint between the two wheel centers.

### 6.5 Differential-Drive Joints

The drive wheels are connected to the chassis using revolute joints.

```text
left_wheel_joint
right_wheel_joint
```

The joints rotate around the local Y-axis in the current SDF configuration.

Gazebo uses these joints to simulate differential-drive motion.

---

## 7. Differential-Drive Kinematics

Differential-drive robots use two independently driven wheels to control translation and rotation.

Let:

- $r$ = Wheel radius (m)
- $L$ = Wheel separation (m)
- $\omega_R$ = Right wheel angular velocity (rad/s)
- $\omega_L$ = Left wheel angular velocity (rad/s)
- $v$ = Robot linear velocity (m/s)
- $\omega$ = Robot angular velocity (rad/s)

### 7.1 Linear Velocity

The robot's forward linear velocity is:

$$
v = \frac{r}{2}(\omega_R + \omega_L)
$$

Using the current wheel radius:

$$
r = 0.02 \text{ m}
$$

Therefore:

$$
v = \frac{0.02}{2}(\omega_R + \omega_L)
$$

$$
\boxed{v = 0.01(\omega_R + \omega_L)}
$$

### 7.2 Angular Velocity

The robot's angular velocity is:

$$
\omega = \frac{r}{L}(\omega_R - \omega_L)
$$

Using:

$$
r = 0.02 \text{ m}
$$

$$
L = 0.10 \text{ m}
$$

The equation becomes:

$$
\omega = \frac{0.02}{0.10}(\omega_R - \omega_L)
$$

$$
\boxed{\omega = 0.2(\omega_R - \omega_L)}
$$

### 7.3 Inverse Kinematics

Given the desired linear velocity $v$ and angular velocity $\omega$, the required wheel angular velocities are:

$$
\omega_R = \frac{v}{r} + \frac{L\omega}{2r}
$$

$$
\omega_L = \frac{v}{r} - \frac{L\omega}{2r}
$$

For the current robot:

$$
\boxed{\omega_R = 50v + 2.5\omega}
$$

$$
\boxed{\omega_L = 50v - 2.5\omega}
$$

These equations can be used to calculate wheel speeds from velocity commands.

### 7.4 Motion Characteristics

**Straight-Line Motion**

When both wheels rotate at the same speed:

$$
\omega_R = \omega_L
$$

The angular velocity becomes:

$$
\omega = 0
$$

The robot moves straight forward or backward.

**Rotation in Place**

When the wheels rotate at equal speeds in opposite directions:

$$
\omega_R = -\omega_L
$$

The linear velocity becomes:

$$
v = 0
$$

The robot rotates around the midpoint of its drive axle.

**Curved Motion**

When the wheel speeds differ:

$$
\omega_R \neq \omega_L
$$

The robot follows a curved trajectory.

---

## 8. Simulation Startup

The project separates simulation startup from Python robot control.

Two independent entry points are provided:

```text
start_sim.sh
main.py
```

### 8.1 Start the Simulation

Open a terminal and navigate to the project directory:

```bash
cd ~/Desktop/Robots
```

Make the startup script executable:

```bash
chmod +x start_sim.sh
```

Start Gazebo and the ROS 2 bridge:

```bash
./start_sim.sh
```

The script performs the following operations:

1. Changes to the project directory.
2. Initializes the ROS 2 Jazzy environment.
3. Configures the Gazebo model search path.
4. Starts Gazebo Harmonic.
5. Loads the simulation world and robot.
6. Starts the ROS 2–Gazebo communication bridge.
7. Keeps the simulation running.

The startup command uses the OGRE rendering engine for improved VMware graphics compatibility.

To stop the simulation, press:

```text
Ctrl+C
```

### 8.2 Start the Python Controller

Open another terminal:

```bash
cd ~/Desktop/Robots
```

Initialize ROS 2 if necessary:

```bash
source /opt/ros/jazzy/setup.bash
```

Run the Python control program:

```bash
/usr/bin/python3 main.py
```

Alternatively, execute `main.py` from PyCharm using the configured Python interpreter.

**Important:** The Gazebo simulation can remain active while the Python controller is restarted or modified.

This enables iterative development without repeatedly loading the simulation world.

---

## 9. ROS 2 and Gazebo Communication

The project uses `ros_gz_bridge` to exchange messages between ROS 2 and Gazebo Transport.

### 9.1 Communication Architecture

```text
Python Application
       |
       v
      rclpy
       |
       v
ROS 2 /cmd_vel
       |
       v
  ros_gz_bridge
       |
       v
Gazebo /cmd_vel
       |
       v
  DiffDrive Plugin
       |
       v
Robot Wheel Joints
       |
       v
  Robot Movement
```

Odometry is transferred in the opposite direction:

```text
Gazebo DiffDrive
       |
       v
Gazebo /odom
       |
       v
  ros_gz_bridge
       |
       v
ROS 2 /odom
       |
       v
Python Application
```

### 9.2 ROS 2 Topics

| Topic | Message Type | Description |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity commands |
| `/odom` | `nav_msgs/msg/Odometry` | Odometry measurements |
| `/camera/image` | `sensor_msgs/msg/Image` | Camera images |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera information |

The current startup configuration bridges `/cmd_vel` and `/odom`.

The camera topics originate in Gazebo and require additional bridge configuration before they can be accessed as ROS 2 topics.

### 9.3 ROS 2 Bridge Configuration

The startup script uses:

```bash
ros2 run ros_gz_bridge parameter_bridge \
    '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
    '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'
```

Bridge direction symbols:

- `]` — ROS 2 to Gazebo
- `[` — Gazebo to ROS 2
- `@` — Bidirectional communication

### 9.4 Inspect Topics

List available ROS 2 topics:

```bash
ros2 topic list
```

Inspect the velocity command topic:

```bash
ros2 topic info /cmd_vel
```

Inspect the odometry topic:

```bash
ros2 topic info /odom
```

---

## 10. Robot Control

The Python robot controller is located at:

```text
src/controller/robot_controller.py
```

The main application entry point is:

```text
main.py
```

### 10.1 Velocity Commands

The robot uses the ROS 2 message type:

```text
geometry_msgs/msg/Twist
```

The primary control variables are:

```text
linear.x
angular.z
```

`linear.x` controls forward and backward movement.

`angular.z` controls rotational movement around the Z-axis.

### 10.2 Forward Movement

Send a forward velocity command:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

The requested forward velocity is:

$$
v = 0.05 \text{ m/s}
$$

### 10.3 Backward Movement

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: -0.05}, angular: {z: 0.0}}"
```

### 10.4 Left Rotation

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 1.0}}"
```

### 10.5 Right Rotation

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: -1.0}}"
```

### 10.6 Stop the Robot

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

### 10.7 Python Controller Interface

The intended Python control interface provides methods such as:

```python
robot.forward(speed=0.05, duration=2.0)

robot.backward(speed=0.05, duration=1.0)

robot.turn_left(speed=1.0, duration=1.57)

robot.turn_right(speed=1.0, duration=1.57)

robot.stop()
```

These methods are intended to publish ROS 2 velocity commands.

Actual motion depends on simulation timing, wheel friction, wheel slip, and the physics engine.

Time-based motion commands should be considered open-loop control unless feedback is explicitly incorporated.

---

## 11. Localization and Odometry

The `src/localization/` directory is intended for localization algorithm development.

The objective is to estimate the robot's position and orientation as it moves through the environment.

### 11.1 Robot Pose

The planar robot pose is represented as:

$$
\mathbf{x} =
\begin{bmatrix}
x \\
y \\
\theta
\end{bmatrix}
$$

Where:

- $x$ = X-coordinate (m)
- $y$ = Y-coordinate (m)
- $\theta$ = Robot heading angle (rad)

### 11.2 Continuous Motion Model

The differential-drive kinematic model is:

$$
\dot{x} = v\cos(\theta)
$$

$$
\dot{y} = v\sin(\theta)
$$

$$
\dot{\theta} = \omega
$$

Where:

- $v$ = Linear velocity
- $\omega$ = Angular velocity
- $\theta$ = Robot orientation

### 11.3 Discrete Odometry Integration

For a sufficiently small time interval $\Delta t$, the robot pose can be approximated using forward Euler integration.

**X-position update:**

$$
x_{t+1} = x_t + v_t\cos(\theta_t)\Delta t
$$

**Y-position update:**

$$
y_{t+1} = y_t + v_t\sin(\theta_t)\Delta t
$$

**Orientation update:**

$$
\theta_{t+1} = \theta_t + \omega_t\Delta t
$$

These equations estimate the next robot pose from the current pose and measured or estimated velocities.

### 11.4 Wheel-Based Odometry

The linear velocity of the right wheel is:

$$
v_R = r\omega_R
$$

The linear velocity of the left wheel is:

$$
v_L = r\omega_L
$$

The forward velocity is:

$$
v = \frac{v_R + v_L}{2}
$$

The angular velocity is:

$$
\omega = \frac{v_R - v_L}{L}
$$

These relationships allow robot motion to be estimated from wheel measurements.

### 11.5 Odometry Uncertainty

Real robots experience uncertainty due to:

- Wheel slip
- Uneven surfaces
- Encoder measurement errors
- Wheel radius inaccuracies
- Mechanical misalignment
- Sensor noise
- Numerical integration errors

A simple noisy velocity model is:

$$
\tilde{v}_t = v_t + \epsilon_{v,t}
$$

$$
\tilde{\omega}_t = \omega_t + \epsilon_{\omega,t}
$$

Where:

- $\tilde{v}_t$ = Noisy linear velocity
- $\tilde{\omega}_t$ = Noisy angular velocity
- $\epsilon_{v,t}$ = Linear velocity noise
- $\epsilon_{\omega,t}$ = Angular velocity noise

For a basic Gaussian noise model:

$$
\epsilon_{v,t} \sim \mathcal{N}(0,\sigma_v^2)
$$

$$
\epsilon_{\omega,t} \sim \mathcal{N}(0,\sigma_\omega^2)
$$

The noise parameters should be selected and documented for each experiment.

### 11.6 Localization Evaluation

Estimated robot poses should be compared with independently obtained simulator ground-truth poses.

The position error at time $t$ is:

$$
e_t = \sqrt{(\hat{x}_t-x_t^{GT})^2 + (\hat{y}_t-y_t^{GT})^2}
$$

Where:

- $\hat{x}_t$ = Estimated X-position
- $\hat{y}_t$ = Estimated Y-position
- $x_t^{GT}$ = Ground-truth X-position
- $y_t^{GT}$ = Ground-truth Y-position

### 11.7 Root Mean Square Error

Localization accuracy can be evaluated using RMSE:

$$
\mathrm{RMSE} = \sqrt{\frac{1}{N}\sum_{t=1}^{N}e_t^2}
$$

Where:

- $N$ = Number of evaluated poses
- $e_t$ = Position error at time $t$

A lower RMSE indicates a smaller average squared localization error.

**Important:** Ground-truth poses must be used only for evaluation and must not directly provide the robot's localization estimates.

---

## 12. Camera and Sensor Configuration

The current robot model includes a simulated front-facing camera.

### Camera Parameters

| Parameter | Value |
|---|---|
| Sensor Name | `front_camera` |
| Sensor Type | Camera |
| Image Width | 640 px |
| Image Height | 480 px |
| Update Rate | 20 Hz |
| Horizontal Field of View | 1.047 rad |
| Near Clipping Distance | 0.01 m |
| Far Clipping Distance | 30 m |
| Gazebo Image Topic | `/camera/image` |

Camera parameters are defined in:

```text
models/maze_robot/model.sdf
```

### Inspect Gazebo Camera Topics

```bash
gz topic -l
```

Inspect the image topic:

```bash
gz topic -i -t /camera/image
```

### Future Camera Integration

The camera is intended to support:

- Wall detection
- Free-space detection
- Obstacle identification
- Visual maze exploration
- Image-based navigation
- Mapping and localization

ROS 2 camera integration requires bridging image and camera information topics.

---

## 13. Simulation Environment

The simulation world is defined in:

```text
world.sdf
```

### Current Environment

The world includes:

- Gravity
- Physics simulation
- Ground collision geometry
- Ground visualization
- Directional lighting
- Robot model
- Gazebo physics system
- Gazebo scene broadcaster
- Gazebo sensor system

### Physics Configuration

The current world uses a maximum physics step size of:

$$
\Delta t = 0.001 \text{ s}
$$

The configured real-time factor is:

$$
RTF = 1.0
$$

This represents a target of one second of simulated time per second of real time. Actual performance may be lower depending on hardware and rendering load.

### Future Environment Features

Planned extensions include:

- Maze generation
- Wall and obstacle placement
- Randomized maze layouts
- Dynamic obstacles
- Multiple starting positions
- Simulation reset and restart
- Environment randomization
- Automated navigation experiments

These features may be implemented using Python together with Gazebo services and ROS 2 interfaces.

---

## 14. Experimental Results

The `results/` directory is reserved for simulation outputs and evaluation data.

### Suggested Structure

```text
results/
├── estimated_trajectory.csv
├── ground_truth.csv
├── odometry_data.csv
├── trajectory_comparison.png
├── localization_error.png
└── experiment_summary.json
```

These are proposed output files and are not automatically generated by the base simulation.

### Recommended Evaluation Metrics

| Metric | Description |
|---|---|
| Position Error | Distance between estimated and actual position |
| Orientation Error | Difference between estimated and actual heading |
| RMSE | Root mean square position error |
| Trajectory Drift | Accumulated localization deviation |
| Navigation Success Rate | Percentage of successful navigation attempts |
| Collision Count | Number of detected collisions |
| Completion Time | Time required to reach the target |

### Trajectory Visualization

The localization evaluation should include a plot containing:

- Ground-truth trajectory
- Estimated trajectory
- X-axis in meters
- Y-axis in meters
- A clearly labelled legend

The comparison can be generated using Python and Matplotlib.

---

## 15. Troubleshooting

### 15.1 Gazebo Cannot Find the Robot

**Error:**

```text
Unable to find uri[model://maze_robot]
```

**Solution:**

Verify the model directory:

```bash
ls ~/Desktop/Robots/models/maze_robot
```

Configure the resource path:

```bash
cd ~/Desktop/Robots

export GZ_SIM_RESOURCE_PATH="$PWD/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
```

Restart Gazebo.

### 15.2 Gazebo Crashes During Rendering

**Possible error:**

```text
VMware: vmw_ioctl_command error Invalid argument
```

A related crash may reference:

```text
libgz-rendering-ogre2.so
RenderSystem_GL3Plus.so
```

**Solution:**

Use the OGRE rendering engine:

```bash
gz sim -r --render-engine ogre world.sdf
```

The provided `start_sim.sh` already uses this configuration.

### 15.3 Python Cannot Import rclpy

**Error:**

```text
ModuleNotFoundError: No module named 'rclpy'
```

**Solution:**

Initialize ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

Run with system Python:

```bash
/usr/bin/python3 main.py
```

Verify that PyCharm uses the correct interpreter and ROS 2 environment.

### 15.4 Robot Does Not Move

Check whether the simulation is running.

Inspect the velocity topic:

```bash
ros2 topic info /cmd_vel
```

Verify that the bridge is running.

Inspect the differential-drive plugin configuration in:

```text
models/maze_robot/model.sdf
```

Confirm that the configured joint names are:

```text
left_wheel_joint
right_wheel_joint
```

### 15.5 No Odometry Data

Check the ROS 2 odometry topic:

```bash
ros2 topic echo /odom
```

Check the Gazebo odometry topic:

```bash
gz topic -i -t /odom
```

Ensure that the bridge includes:

```text
/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry
```

### 15.6 Incorrect Model Appearance

Possible causes include:

- Missing mesh files
- Incorrect mesh URIs
- Incorrect material export
- Incorrect component origins
- Incorrect scaling
- Unsupported material settings

Verify that the visual mesh paths in `model.sdf` match the actual files in `models/maze_robot/meshes/`.

### 15.7 Conda and ROS 2 Compatibility

ROS 2 Jazzy is installed against the system Python environment on Ubuntu 24.04.

Using an incompatible Conda environment may cause import or shared-library errors.

For initial testing, use:

```bash
/usr/bin/python3
```

---

## 16. Development Roadmap

### Phase 1 — Robot Modelling and Simulation

- [x] Design the robot in Blender
- [x] Export robot mesh components
- [x] Create the SDF robot model
- [x] Define wheel joints
- [x] Configure differential-drive geometry
- [x] Add a simulated camera
- [x] Create the Gazebo world
- [x] Load the robot in Gazebo
- [x] Resolve VMware rendering compatibility
- [x] Create a dedicated simulation startup script

### Phase 2 — ROS 2 Robot Control

- [ ] Verify ROS 2–Gazebo bridge communication
- [ ] Verify forward and backward motion
- [ ] Verify rotational motion
- [ ] Implement reusable Python motion methods
- [ ] Implement closed-loop movement control
- [ ] Add interactive keyboard control
- [ ] Record control commands and movement data

### Phase 3 — Localization

- [ ] Implement differential-drive odometry
- [ ] Integrate wheel motion measurements
- [ ] Add simulated measurement uncertainty
- [ ] Estimate robot position and orientation
- [ ] Collect independent ground-truth data
- [ ] Compare estimated and actual trajectories
- [ ] Calculate localization RMSE
- [ ] Generate localization plots

### Phase 4 — Environment Perception

- [ ] Bridge camera topics to ROS 2
- [ ] Read camera images using Python
- [ ] Implement wall and obstacle detection
- [ ] Identify navigable space
- [ ] Explore additional perception algorithms

### Phase 5 — Autonomous Maze Navigation

- [ ] Generate configurable maze environments
- [ ] Implement maze exploration
- [ ] Integrate SLAM
- [ ] Implement path planning
- [ ] Implement autonomous obstacle avoidance
- [ ] Evaluate navigation efficiency
- [ ] Compare navigation algorithms

---

## 17. Development Workflow

The recommended workflow separates environment execution from algorithm development.

### Step 1 — Start the Simulation

```bash
cd ~/Desktop/Robots
./start_sim.sh
```

### Step 2 — Start the Python Controller

```bash
cd ~/Desktop/Robots
/usr/bin/python3 main.py
```

### Step 3 — Develop and Test

Modify the Python modules in:

```text
src/controller/
src/localization/
```

Rerun `main.py` without restarting the Gazebo environment.

### Step 4 — Record Results

Save experimental data in:

```text
results/
```

### Step 5 — Commit Changes

```bash
git status
git add .
git commit -m "Implement robot control improvements"
```

Keep commits small, meaningful, and descriptive.

---

## 18. References

### ROS 2

ROS 2 Jazzy Documentation:

https://docs.ros.org/en/jazzy/

ROS 2 Python Client Library:

https://docs.ros.org/en/jazzy/p/rclpy/

### Gazebo

Gazebo Harmonic Documentation:

https://gazebosim.org/docs/harmonic/

Gazebo ROS 2 Integration:

https://gazebosim.org/docs/harmonic/ros2_integration/

### Robot Modelling

Blender:

https://www.blender.org/

SDF Specification:

http://sdformat.org/

### Robotics and Localization

Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.

Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.

---

## 19. Project Status

| Subsystem | Status |
|---|---|
| Blender Robot Model | Completed |
| SDF Robot Description | Completed |
| Gazebo Environment | Running |
| OGRE Rendering Compatibility | Verified |
| Differential-Drive Plugin Configuration | Completed |
| ROS 2 Bridge Configuration | Prepared |
| Python Control Integration | In Development |
| Odometry Evaluation | Planned |
| Localization Algorithms | Planned |
| Camera-Based Perception | Planned |
| SLAM Integration | Planned |
| Autonomous Maze Navigation | Planned |

**Current Development Focus:** ROS 2 Python control integration and differential-drive localization.

---

**Project:** Maze Robot — Intelligent Robotics

**Platform:** Ubuntu 24.04 LTS / ROS 2 Jazzy / Gazebo Harmonic

**Language:** Python

**Simulation Model:** Differential-Drive Mobile Robot

**Primary Goal:** Autonomous navigation and localization in unknown maze environments.