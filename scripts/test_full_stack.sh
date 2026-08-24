#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /home/saad/kerabot_ws/install/setup.bash

killall -9 gzserver gzclient rosmaster python3 2>/dev/null || true
sleep 1

echo "1. Launching Gazebo + Robot + Controllers..."
ros2 launch /home/saad/kerabot_ws/scripts/test_gazebo_full.launch.py &
GZ_PID=$!

echo "2. Waiting for arm_controller to become active..."
for i in {1..30}; do
    if ros2 control list_controllers 2>/dev/null | grep -q "arm_controller.*active"; then
        echo "SUCCESS: arm_controller is active after $i seconds!"
        break
    fi
    sleep 1
done

echo "3. Launching MoveGroup..."
ros2 launch kerabot_moveit_config move_group.launch.py &
MG_PID=$!

echo "4. Waiting for MoveGroup planning scene..."
sleep 5

echo "5. Verifying /joint_states and controllers..."
ros2 topic echo /joint_states --once
ros2 control list_controllers

echo "6. Running 6-DoF Peeling Benchmark Suite..."
cd /home/saad/kerabot_ws
python3 scripts/peel_place_benchmark_suite.py

echo "Cleaning up..."
kill -9 $GZ_PID $MG_PID 2>/dev/null || true
killall -9 gzserver gzclient python3 2>/dev/null || true
