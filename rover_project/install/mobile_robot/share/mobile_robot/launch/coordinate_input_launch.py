import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mobile_robot',  # Replace with your actual package name
            executable='coordinate_input_node',
            name='coordinate_input_node',
            output='screen',
        ),
    ])