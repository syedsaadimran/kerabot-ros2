import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_description = get_package_share_directory('Robot_to_URDF_New_Pakka_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_moveit_config = get_package_share_directory('kerabot_moveit_config')

    # Paths
    default_world = os.path.join(pkg_description, 'worlds', 'sticker_workcell.world')
    xacro_file = os.path.join(pkg_description, 'urdf', 'Robot_to_URDF_New_Pakka.urdf.xacro')
    controllers_file = os.path.join(pkg_moveit_config, 'config', 'ros2_controllers.yaml')

    # Launch configurations
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    pause = LaunchConfiguration('pause')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Generate robot_description from xacro with use_gazebo:=true
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_gazebo:=true',
            ' gazebo_controllers:=', controllers_file,
        ]),
        value_type=str
    )

    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world,
            'pause': pause,
        }.items(),
    )

    # Gazebo client (GUI)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={'gui': gui}.items(),
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # Spawn Robot Entity into Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_kerabot',
        output='screen',
        arguments=[
            '-entity', 'Robot_to_URDF_New_Pakka',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Full path to world model file to load',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Set to true to open Gazebo graphical interface',
        ),
        DeclareLaunchArgument(
            'pause',
            default_value='false',
            description='Set to true to start Gazebo in a paused state',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock from Gazebo /clock topic',
        ),

        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
    ])
