#!/bin/bash

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source /home/arm2025/drive_wa01/install/setup.bash

echo "Starting Ackermann Node..."
# Start the ackermann driver node in the background
ros2 launch drive_controller ackermann.launch.py &
ACKERMANN_PID=$!

# Ensure the ackermann node is stopped if this script is closed
trap "echo 'Stopping Ackermann Node...'; kill $ACKERMANN_PID" EXIT

# Wait briefly to let the ackermann node start
sleep 3

echo "Starting Keyboard Teleop Node..."
# Start the keyboard teleop node (this opens an xterm window and blocks until closed)
ros2 launch drive_controller keyboard_teleop.launch.py
