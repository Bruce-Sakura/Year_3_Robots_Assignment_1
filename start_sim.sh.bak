#!/usr/bin/env bash
set -e

# 进入启动脚本所在目录
cd "$(dirname "$(realpath "$0")")"

# 初始化 ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Gazebo 模型目录
ROBOT_DIR="$PWD/Round_Robot"

# Gazebo 模型搜索路径
export GZ_SIM_RESOURCE_PATH="$ROBOT_DIR/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

echo "Starting Gazebo Harmonic..."

gz sim -r --render-engine ogre "$ROBOT_DIR/world.sdf" &
GAZEBO_PID=$!

echo "Starting ROS 2 Bridge..."

# ROS /cmd_vel -> Gazebo DiffDrive; Gazebo odom and wheel encoders -> ROS
ros2 run ros_gz_bridge parameter_bridge \
    '/model/round_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
    '/model/round_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
    '/model/round_robot/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model' \
    --ros-args \
    -r /model/round_robot/cmd_vel:=/cmd_vel \
    -r /model/round_robot/odometry:=/odom \
    -r /model/round_robot/joint_states:=/joint_states &
BRIDGE_PID=$!

cleanup() {
    trap - EXIT INT TERM
    echo "Stopping simulation..."
    kill "$BRIDGE_PID" "$GAZEBO_PID" 2>/dev/null || true
    wait "$BRIDGE_PID" "$GAZEBO_PID" 2>/dev/null || true
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "Gazebo and ROS 2 Bridge started."
echo "Press Ctrl+C to stop."

wait "$GAZEBO_PID"
