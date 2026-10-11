Round robot model with tread blocks and an elevated concept board

The Raspberry Pi 6 board is a conceptual placeholder, using an 85 x 56 mm Pi 5 envelope and 58 x 49 mm mounting-hole spacing. Ports and chips are simplified visual representations; the model does not establish official Pi 6 mechanical dimensions.
Four 28 mm standoffs support the board. Its underside is at z = 0.018 m, 12 mm above the motor tops.
The tyres have 48 pairs of raised tread blocks, with an effective rolling radius of approximately 25 mm. The simulation uses cylindrical wheel collision shapes of radius 25 mm.
The visual tyre width is 12 mm. The checked-in SDF collision width was reduced to 1 mm during contact diagnosis; visual dimensions and collision dimensions are distinct.
Motor wiring passes through the driver board. GPIO and power wires are visual illustrations; no battery, electrical circuit, or thermal design is included.
The chassis has a nominal diameter of 100 mm and thickness of 5 mm, with 2 mm wheel-slot clearance on each side.
Wheel diameter is 50 mm and centre-to-centre separation is 100 mm. Vertical axle brackets have 4.6 mm holes. Motors illustrate dimensions and do not represent a specified commercial model.
The axle midpoint/base_link origin is (0, 0, 0). The chassis geometric centre/model_center is (0, 0, -0.0125) m. Their horizontal positions coincide, but their heights differ by 12.5 mm.
The wheel bottoms are at z = -0.025 m; the chassis underside is at z = -0.015 m, giving 10 mm clearance. Front and rear steel balls have diameter 10 mm and centres at (+/-0.036, 0, -0.020) m, with the same lowest point as the wheels.
Open round_robot.blend to inspect independent parts in metre units with applied rotation and scale. build_robot.py runs inside Blender and overwrites generated meshes, parts.json, the Blender scene, and previews; it does not rebuild the SDF or preserve manual mesh edits.
From the repository root, run bash start_sim.sh to launch Gazebo and ROS bridges, then python main.py in a ROS-enabled terminal to execute the three waypoints. Running python3 Round_Robot/main.py launches Gazebo only, without ROS bridges or navigation.
Gazebo drive topics are /model/round_robot/cmd_vel and /model/round_robot/odometry. ROS uses /cmd_vel, /odom, /joint_states, and /ground_truth; ground truth is for evaluation only.
For r = 0.025 m and b = 0.10 m: v = 0.0125 * (wR + wL) and yaw_rate = 0.25 * (wR - wL), with wheel speeds in rad/s.
Masses and inertias are estimates. The chassis uses simplified collision geometry, caster balls use ball joints, and motors and brackets are visual parts. Stored experiment records are available, but this documentation update does not constitute a new dynamics validation.
