import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_description = get_package_share_directory('Robot_to_URDF_New_Pakka_description')
    pkg_moveit_config = get_package_share_directory('kerabot_moveit_config')

    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')

    # 1. Include Gazebo robot spawn launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'gui': gui, 'use_sim_time': 'true'}.items(),
    )

    # 2. MoveIt Configurations (with use_sim_time enabled)
    moveit_config = (
        MoveItConfigsBuilder(
            "Robot_to_URDF_New_Pakka",
            package_name="kerabot_moveit_config",
        )
        .robot_description(
            file_path=os.path.join(
                pkg_description, "urdf", "Robot_to_URDF_New_Pakka.urdf.xacro"
            ),
            mappings={
                "use_gazebo": "true",
                "gazebo_controllers": os.path.join(
                    pkg_moveit_config, "config", "ros2_controllers.yaml"
                ),
            },
        )
        .planning_pipelines(
            default_planning_pipeline="pilz_industrial_motion_planner",
            pipelines=["ompl", "pilz_industrial_motion_planner"],
        )
        .to_moveit_configs()
    )

    # 3. MoveGroup Node with simulation time
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    # 4. RViz2 Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(pkg_moveit_config, "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    # 5. Controller Spawners
    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawner_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Sequence spawning after joint_state_broadcaster activates
    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawner_jsb,
            on_exit=[spawner_arm],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Launch Gazebo GUI window',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz visualization window',
        ),

        gazebo_launch,
        spawner_jsb,
        delay_arm_controller,
        move_group_node,
        rviz_node,
    ])
