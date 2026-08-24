from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("Robot_to_URDF_New_Pakka", package_name="kerabot_moveit_config")
        .to_moveit_configs()
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
                "trajectory_execution.allowed_start_tolerance": 0.0,
                "trajectory_execution.allowed_execution_duration_scaling": 2.0,
                "trajectory_execution.allowed_goal_duration_margin": 1.0,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        ),
        move_group_node,
    ])
