from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mobile_robot',
            executable='coordinate_input_node',
            name='coordinate_input_node',
            output='screen',
        ),
        Node(
            package='mobile_robot',
            executable='gps_subscriber',
            name='gps_subscriber',
            output='screen',
        ),
        Node(
            package='mobile_robot',
            executable='rover_control_node',
            name='rover_control_node',
            output='screen',
        ),
    ])
