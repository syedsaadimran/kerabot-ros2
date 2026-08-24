import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description = get_package_share_directory('Robot_to_URDF_New_Pakka_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_moveit_config = get_package_share_directory('kerabot_moveit_config')

    # Paths
    default_world = os.path.join(pkg_description, 'worlds', 'sticker_workcell.world')
    gazebo_urdf_file = os.path.join(pkg_description, 'urdf', 'Robot_to_URDF_New_Pakka_gazebo.urdf')
    controllers_file = os.path.join(pkg_moveit_config, 'config', 'ros2_controllers.yaml')

    # Launch configurations
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Read clean URDF content
    with open(gazebo_urdf_file, 'r') as f:
        robot_description_content = f.read()

    # Disable online model lookup hang
    disable_model_db = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=''
    )

    # Ensure GAZEBO_MODEL_PATH resolves mesh files
    model_paths = [
        os.path.join(os.path.expanduser('~'), 'kerabot_ws', 'src'),
        os.path.join(os.path.expanduser('~'), 'kerabot_ws', 'install', 'Robot_to_URDF_New_Pakka_description', 'share'),
        os.environ.get('GAZEBO_MODEL_PATH', '')
    ]
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=':'.join([p for p in model_paths if p])
    )

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            default_world,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_force_system.so',
            '--verbose',
        ],
        output='screen',
    )

    # Gazebo client (GUI)
    gzclient = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen',
        condition=IfCondition(gui),
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
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
            '-file', gazebo_urdf_file,
            '-timeout', '120.0',
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
            'use_sim_time',
            default_value='true',
            description='Use simulation clock from Gazebo /clock topic',
        ),

        disable_model_db,
        set_gazebo_model_path,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
    ])
