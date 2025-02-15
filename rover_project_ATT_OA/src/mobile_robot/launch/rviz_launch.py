import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package configuration
    pkg_name = 'mobile_robot'
    default_rviz_config = PathJoinSubstitution([
        get_package_share_directory(pkg_name),
        'rviz',
        'rover_navigation.rviz'  # More descriptive config name
    ])

    # Launch arguments
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to RViz config file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # RViz2 node configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rover_visualization',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        # Uncomment for multi-robot scenarios
        # namespace=LaunchConfiguration('namespace'),
        remappings=[
            # Example remappings if needed:
            # ('/tf', 'tf'),
            # ('/tf_static', 'tf_static')
        ]
    )

    return LaunchDescription([
        rviz_config_arg,
        use_sim_time_arg,
        rviz_node
    ])