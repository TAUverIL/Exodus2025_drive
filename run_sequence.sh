#!/bin/bash

# Start the continuous velocity publisher for the wheels in the background
echo "Starting continuous wheel movement..."
ros2 topic pub /drive_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0, 1.0, 1.0, 1.0]" > /dev/null 2>&1 &
WHEEL_PID=$!

# Ensure the background process is killed when the script exits or is interrupted
trap "echo 'Stopping wheels...'; kill $WHEEL_PID" EXIT

echo "1. Moving pivots to position 0."
ros2 topic pub /pivot_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0]" -1 > /dev/null 2>&1

echo "2. Waiting 5 seconds..."
sleep 3

echo "3. Moving pivots to 45 degrees (0.7854 rad)."
ros2 topic pub /pivot_controller/commands std_msgs/msg/Float64MultiArray "data: [0.7854, 0.7854]" -1 > /dev/null 2>&1

echo "4. Waiting 5 seconds..."
sleep 3

echo "5. Moving pivots to position 0."
ros2 topic pub /pivot_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0]" -1 > /dev/null 2>&1

# Adding a brief delay here so the motors have time to reach 0 before receiving the next command
echo "   (Waiting 2 seconds to allow motors to reach 0)"
sleep 3

echo "6. Moving pivots to -45 degrees (-0.7854 rad) (other direction)."
ros2 topic pub /pivot_controller/commands std_msgs/msg/Float64MultiArray "data: [-0.7854, -0.7854]" -1 > /dev/null 2>&1

echo "7. Waiting 5 seconds..."
sleep 3

echo "Plan complete! Exiting..."
