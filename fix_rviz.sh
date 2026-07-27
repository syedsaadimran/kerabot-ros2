#!/bin/bash

# 1. Kill any hung RViz processes
echo "Cleaning up processes..."
killall -9 rviz2 robot_state_publisher joint_state_publisher_gui 2>/dev/null

# 2. Force software rendering to fix the black screen/copy mode issue
export LIBGL_ALWAYS_SOFTWARE=1

# 3. Source the workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# 4. Launch
echo "Launching RViz..."
ros2 launch Robot_to_URDF_New_Pakka_description display.launch.py
