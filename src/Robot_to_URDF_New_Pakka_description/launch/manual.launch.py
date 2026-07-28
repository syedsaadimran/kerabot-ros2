import os
import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('Robot_to_URDF_New_Pakka_description')
    
    # Dynamically find the URDF file
    urdf_dir = os.path.join(pkg_share, 'urdf')
    urdf_file = glob.glob(os.path.join(urdf_dir, '*.urdf'))[0]

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Node 1: Broadcasts the robot's physical structure
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(robot_desc, value_type=str)}]
        ),
        # Node 2: ONLY the GUI slider (No ros2_control broadcasters allowed!)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        # Node 3: The 3D Visualizer
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'display.rviz')]
        )
    ])
