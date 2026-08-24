#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /home/saad/kerabot_ws/install/setup.bash

killall -9 gzserver gzclient rosmaster python3 2>/dev/null || true
sleep 1

echo "Starting Gazebo + MoveIt2 pipeline (headless)..."
ros2 launch kerabot_moveit_config gazebo_moveit.launch.py gui:=false use_rviz:=false &
LAUNCH_PID=$!

echo "Waiting for MoveGroup and Controllers to become active..."
for i in {1..30}; do
    if ros2 topic list | grep -q "/joint_states"; then
        CONTROLLER_STATUS=$(ros2 control list_controllers 2>/dev/null || true)
        if echo "$CONTROLLER_STATUS" | grep -q "arm_controller.*active"; then
            echo "SUCCESS: arm_controller is active after $i seconds!"
            break
        fi
    fi
    sleep 1
done

echo "Checking MoveGroup status..."
ros2 topic echo /joint_states --once

echo "Running 6-DoF Sticker Peel-and-Place Benchmark Suite in Gazebo simulation..."
cd /home/saad/kerabot_ws
python3 scripts/peel_place_benchmark_suite.py

echo "Benchmark execution completed! Cleaning up..."
kill -9 $LAUNCH_PID 2>/dev/null || true
killall -9 gzserver gzclient python3 2>/dev/null || true
