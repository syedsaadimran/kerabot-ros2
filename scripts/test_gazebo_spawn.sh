#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /home/saad/kerabot_ws/install/setup.bash

killall -9 gzserver gzclient rosmaster python3 2>/dev/null || true
sleep 1

export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH="/home/saad/kerabot_ws/src:/home/saad/kerabot_ws/install/Robot_to_URDF_New_Pakka_description/share"

URDF_FILE="/home/saad/kerabot_ws/install/Robot_to_URDF_New_Pakka_description/share/Robot_to_URDF_New_Pakka_description/urdf/Robot_to_URDF_New_Pakka_gazebo.urdf"
WORLD_FILE="/home/saad/kerabot_ws/install/Robot_to_URDF_New_Pakka_description/share/Robot_to_URDF_New_Pakka_description/worlds/sticker_workcell.world"

# 1. Start gzserver
echo "1. Starting gzserver..."
gzserver "$WORLD_FILE" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so -s libgazebo_ros_force_system.so --verbose &
GZ_PID=$!

# 2. Start robot_state_publisher
echo "2. Starting robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher "$URDF_FILE" --ros-args -p use_sim_time:=true &
RSP_PID=$!

# 3. Wait for /spawn_entity service
echo "3. Waiting for Gazebo spawn service..."
for i in {1..30}; do
    if ros2 service list | grep -q "spawn_entity"; then
        echo "Found /spawn_entity at $i seconds!"
        break
    fi
    sleep 1
done

# 4. Spawn Robot Entity
echo "4. Spawning Robot_to_URDF_New_Pakka into Gazebo..."
ros2 run gazebo_ros spawn_entity.py -entity Robot_to_URDF_New_Pakka -file "$URDF_FILE" -timeout 60.0

# 5. Wait for controller_manager
echo "5. Waiting for controller_manager service..."
for i in {1..30}; do
    if ros2 service list | grep -q "/controller_manager/list_controllers"; then
        echo "Found /controller_manager at $i seconds!"
        break
    fi
    sleep 1
done

# 6. Spawn controllers
echo "6. Spawning joint_state_broadcaster & arm_controller..."
ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /controller_manager --controller-manager-timeout 30 || true
ros2 run controller_manager spawner arm_controller --controller-manager /controller_manager --controller-manager-timeout 30 || true

echo "7. Verifying active controllers in Gazebo:"
ros2 control list_controllers

echo "8. Verifying /joint_states publication:"
ros2 topic echo /joint_states --once

echo "SUCCESS! Pipeline verified."
kill -9 $GZ_PID $RSP_PID 2>/dev/null || true
killall -9 gzserver gzclient python3 2>/dev/null || true
