import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(file_path):
    if os.path.exists(file_path):
        with open(file_path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    return {}


def load_file(file_path):
    if os.path.exists(file_path):
        with open(file_path, 'r', encoding='utf-8') as f:
            return f.read()
    return ''


def generate_launch_description():
    pkg_description = get_package_share_directory('Robot_to_URDF_New_Pakka_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_moveit_config = get_package_share_directory('kerabot_moveit_config')

    # Paths
    world_file = os.path.join(pkg_description, 'worlds', 'sticker_workcell.world')
    gazebo_urdf_file = os.path.join(pkg_description, 'urdf', 'Robot_to_URDF_New_Pakka_gazebo.urdf')
    controllers_file = os.path.join(pkg_moveit_config, 'config', 'ros2_controllers.yaml')
    srdf_file = os.path.join(pkg_moveit_config, 'config', 'Robot_to_URDF_New_Pakka.srdf')
    kinematics_file = os.path.join(pkg_moveit_config, 'config', 'kinematics.yaml')
    joint_limits_file = os.path.join(pkg_moveit_config, 'config', 'joint_limits.yaml')
    ompl_planning_file = os.path.join(pkg_moveit_config, 'config', 'ompl_planning.yaml')
    pilz_planning_file = os.path.join(pkg_moveit_config, 'config', 'pilz_industrial_motion_planner_planning.yaml')
    cartesian_limits_file = os.path.join(pkg_moveit_config, 'config', 'pilz_cartesian_limits.yaml')
    moveit_controllers_file = os.path.join(pkg_moveit_config, 'config', 'moveit_controllers.yaml')

    # Launch Configurations
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    world = LaunchConfiguration('world')

    # Read clean URDF & SRDF
    robot_description_content = load_file(gazebo_urdf_file)
    robot_description_semantic_content = load_file(srdf_file)

    # Environment variables
    disable_model_db = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=''
    )

    model_paths = [
        os.path.join(os.path.expanduser('~'), 'kerabot_ws', 'src'),
        os.path.join(os.path.expanduser('~'), 'kerabot_ws', 'install', 'Robot_to_URDF_New_Pakka_description', 'share'),
        os.environ.get('GAZEBO_MODEL_PATH', '')
    ]
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=':'.join([p for p in model_paths if p])
    )

    # 1. Gazebo Server
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

    # 2. Gazebo Client (GUI)
    gzclient = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen',
        condition=IfCondition(gui),
    )

    # 3. Robot State Publisher
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

    # 4. Spawn Robot Entity into Gazebo
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

    # 5. MoveIt Configuration Dictionary
    moveit_controllers = load_yaml(moveit_controllers_file)
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 2.0,
        'trajectory_execution.allowed_goal_duration_margin': 1.0,
        'trajectory_execution.allowed_start_tolerance': 0.0,
    }

    planning_pipelines_config = {
        'default_planning_pipeline': 'pilz_industrial_motion_planner',
        'pipeline_names': ['ompl', 'pilz_industrial_motion_planner'],
        'ompl': load_yaml(ompl_planning_file),
        'pilz_industrial_motion_planner': load_yaml(pilz_planning_file),
    }

    move_group_params = [
        {'robot_description': robot_description_content},
        {'robot_description_semantic': robot_description_semantic_content},
        {'robot_description_kinematics': load_yaml(kinematics_file)},
        {'robot_description_planning': load_yaml(joint_limits_file)},
        {'robot_description_planning': load_yaml(cartesian_limits_file)},
        planning_pipelines_config,
        trajectory_execution,
        moveit_controllers,
        {'use_sim_time': True},
    ]

    # 6. MoveGroup Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )

    # 7. RViz2 Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(pkg_moveit_config, "config", "moveit.rviz")],
        parameters=[
            {'robot_description': robot_description_content},
            {'robot_description_semantic': robot_description_semantic_content},
            {'robot_description_kinematics': load_yaml(kinematics_file)},
            planning_pipelines_config,
            {'use_sim_time': True},
        ],
        condition=IfCondition(use_rviz),
    )

    # 8. Controller Spawners (sequenced after robot spawn)
    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    spawner_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    delay_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[spawner_jsb],
        )
    )

    delay_arm = RegisterEventHandler(
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
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='Full path to world model file to load',
        ),

        disable_model_db,
        set_model_path,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
        delay_jsb,
        delay_arm,
        move_group_node,
        rviz_node,
    ])
