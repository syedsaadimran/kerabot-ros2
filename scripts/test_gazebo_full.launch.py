import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description = get_package_share_directory('Robot_to_URDF_New_Pakka_description')
    pkg_moveit_config = get_package_share_directory('kerabot_moveit_config')

    world_file = os.path.join(pkg_description, 'worlds', 'sticker_workcell.world')
    gazebo_urdf_file = os.path.join(pkg_description, 'urdf', 'Robot_to_URDF_New_Pakka_gazebo.urdf')
    controllers_file = os.path.join(pkg_moveit_config, 'config', 'ros2_controllers.yaml')

    with open(gazebo_urdf_file, 'r') as f:
        robot_description_content = f.read()

    # Disable slow online model lookup
    disable_model_db = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=''
    )

    # Add package sources to GAZEBO_MODEL_PATH for mesh loading
    model_paths = [
        os.path.join(os.path.expanduser('~'), 'kerabot_ws', 'src'),
        os.path.join(os.path.expanduser('~'), 'kerabot_ws', 'install', 'Robot_to_URDF_New_Pakka_description', 'share'),
        os.environ.get('GAZEBO_MODEL_PATH', '')
    ]
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=':'.join([p for p in model_paths if p])
    )

    # 1. Start gzserver
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_force_system.so',
            '--verbose',
        ],
        output='screen',
    )

    # 2. Robot State Publisher with clean in-memory parameter
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }],
    )

    # 3. Spawn Robot Entity into Gazebo from file
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

    # 4. Controller Spawners (Delayed after spawn_robot)
    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    spawner_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    # Trigger JSB when spawn completes
    delay_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[spawner_jsb],
        )
    )

    # Trigger ARM controller when JSB completes
    delay_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawner_jsb,
            on_exit=[spawner_arm],
        )
    )

    return LaunchDescription([
        disable_model_db,
        set_model_path,
        gzserver,
        robot_state_publisher,
        spawn_robot,
        delay_jsb,
        delay_arm,
    ])
