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

echo "=========================================================="
echo " Starting Gazebo Simulation Pipeline & MoveIt2 Interface  "
echo "=========================================================="

# 1. Start Gazebo Server
echo "[1/6] Launching gzserver (headless)..."
gzserver "$WORLD_FILE" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so -s libgazebo_ros_force_system.so --verbose &
GZ_PID=$!

# 2. Start Robot State Publisher
echo "[2/6] Launching robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher "$URDF_FILE" --ros-args -p use_sim_time:=true &
RSP_PID=$!

# 3. Wait for /spawn_entity service
echo "[3/6] Waiting for Gazebo /spawn_entity service..."
for i in {1..30}; do
    if ros2 service list | grep -q "spawn_entity"; then
        echo " -> Gazebo /spawn_entity ready ($i s)"
        break
    fi
    sleep 1
done

# 4. Spawn Robot Entity into Gazebo
echo "[4/6] Spawning 6-DoF Kerabot entity into Gazebo..."
ros2 run gazebo_ros spawn_entity.py -entity Robot_to_URDF_New_Pakka -file "$URDF_FILE" -timeout 60.0

# 5. Activate Controllers
echo "[5/6] Activating ros2_control hardware controllers..."
for i in {1..30}; do
    if ros2 service list | grep -q "/controller_manager/list_controllers"; then
        break
    fi
    sleep 1
done

ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /controller_manager --controller-manager-timeout 30 || true
ros2 run controller_manager spawner arm_controller --controller-manager /controller_manager --controller-manager-timeout 30 || true

echo " -> Active Controllers:"
ros2 control list_controllers

echo " -> Commanding standby posture..."
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{joint_names: ["Revolute_1", "Revolute_2", "Revolute_3", "Revolute_4", "Revolute_5", "ee_rotation_joint"], points: [{positions: [0.0, -0.5, 1.0, 0.0, -0.5, 0.0], time_from_start: {sec: 1, nanosec: 0}}]}' >/dev/null 2>&1
sleep 2

# 6. Launch MoveGroup
echo "[6/6] Launching MoveGroup node..."
ros2 launch kerabot_moveit_config move_group.launch.py &
MG_PID=$!

sleep 6

echo "=========================================================="
echo " Running 6-DoF Peeling Benchmark Suite in Gazebo           "
echo "=========================================================="
cd /home/saad/kerabot_ws
python3 scripts/peel_place_benchmark_suite.py

echo "Cleaning up processes..."
kill -9 $GZ_PID $RSP_PID $MG_PID 2>/dev/null || true
killall -9 gzserver gzclient python3 2>/dev/null || true
echo "Pipeline execution completed successfully!"
