from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'namespace',
            default_value='navigation_system',
            description='Namespace for the navigation nodes'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level (debug, info, warn, error)'
        ),
        
        # Coordinate Input Node
        Node(
            package='mobile_robot',
            executable='coordinate_input_node',
            name='coordinate_input',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                {'target_frame': 'map'},
                {'max_retries': 3},
                {'input_timeout': 10.0}
            ],
            remappings=[
                ('/target_bearing', 'target_bearing'),
                ('/gps', 'gps/filtered')
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
        
        # GPS Subscriber Node with parameters
        Node(
            package='mobile_robot',
            executable='gps_subscriber',
            name='gps_processor',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('mobile_robot'),
                    'config',
                    'gps_params.yaml'
                ]),
                {'use_sim_time': True},
                {'publish_rate': 1.0}
            ],
            remappings=[
                ('/gps', 'gps/raw'),
                ('/gps/filtered', 'gps/processed')
            ]
        ),
        
        # Rover Control Node with lifecycle configuration
        Node(
            package='mobile_robot',
            executable='rover_control_node',
            name='motion_controller',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                {'kp_angular': 0.8},
                {'ki_angular': 0.01},
                {'kd_angular': 0.1},
                {'max_linear_speed': 0.5},
                {'min_linear_speed': 0.1},
                {'navigation_timeout': 30.0}
            ],
            remappings=[
                ('/cmd_vel', 'cmd_vel_nav'),
                ('/odom', 'wheel_odometry')
            ],
            on_exit=[LogInfo(msg="Navigation controller exited")]
        ),
        
        # Add lifecycle manager if needed
        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_navigation',
        #     output='screen',
        #     parameters=[{'autostart': True},
        #                {'node_names': ['motion_controller']}]
        # )
    ])