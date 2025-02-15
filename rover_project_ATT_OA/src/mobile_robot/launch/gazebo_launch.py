import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro

def generate_launch_description():
    # Package and Path Configuration
    pkg_name = 'mobile_robot'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Launch Arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    world_file = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([pkg_share, 'worlds', 'mars_yard.world']),
        description='Path to Gazebo world file'
    )
    
    # Robot Description Processing
    xacro_path = os.path.join(pkg_share, 'urdf', 'rover.xacro')
    robot_description_content = Command(['xacro ', xacro_path, ' use_gazebo:=true'])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Gazebo Launch with Configuration
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world_file'),
            'verbose': 'false',
            'pause': 'false',
            'gui': 'true',
            'extra_gazebo_args': '--ros-args --log-level error'
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Spawn Entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'rover',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # Joint State Broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen'
    )

    # Rover Controllers
    rover_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rover_base_controller', '-c', '/controller_manager'],
        output='screen'
    )

    # Navigation Stack Components
    navigation_nodes = [
        Node(
            package=pkg_name,
            executable='coordinate_input_node',
            name='navigation_input',
            parameters=[PathJoinSubstitution([pkg_share, 'config', 'navigation_params.yaml'])],
            remappings=[('/cmd_vel', 'cmd_vel_nav')]
        ),
        Node(
            package=pkg_name,
            executable='gps_subscriber',
            name='gps_processor',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        Node(
            package=pkg_name,
            executable='rover_control_node',
            name='motion_controller',
            parameters=[PathJoinSubstitution([pkg_share, 'config', 'control_params.yaml'])],
            remappings=[('/odom', 'wheel_odometry')]
        ),
        Node(
            package=pkg_name,
            executable='obstacle_avoidance',
            name='safety_controller',
            parameters=[PathJoinSubstitution([pkg_share, 'config', 'safety_params.yaml'])],
            remappings=[('/cmd_vel', 'cmd_vel_safety')]
        )
    ]

    # RViz2 with Configuration
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'rover_navigation.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # Event Handlers
    load_joint_state_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster]
        )
    )

    load_rover_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[rover_controller]
        )
    )

    return LaunchDescription([
        use_sim_time,
        world_file,
        gazebo_launch,
        robot_state_publisher,
        spawn_entity,
        load_joint_state_controller,
        load_rover_controller,
        rviz_node,
        *navigation_nodes
    ])