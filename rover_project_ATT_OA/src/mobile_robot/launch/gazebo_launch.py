import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Package and file paths
    namePackage = 'mobile_robot'
    modelFileRelativePath = 'model/rover.xacro'
    worldFileRelativePath = 'model/world.world'

    # Construct absolute file paths
    package_share_directory = get_package_share_directory(namePackage)
    pathModelFile = os.path.join(package_share_directory, modelFileRelativePath)
    pathWorldFile = os.path.join(package_share_directory, worldFileRelativePath)

    # Process the Xacro file into XML
    robot_description_config = xacro.process_file(pathModelFile)
    robot_description = robot_description_config.toxml()

    # Include Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': pathWorldFile}.items()
    )

    # Node to publish the robot's URDF to TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # Spawn model in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'rover', '-file', pathModelFile],
        output='screen'
    )

    # Coordinate Input Node (for manual goal setting)
    coordinate_input_node = Node(
        package='mobile_robot',
        executable='coordinate_input_node',
        output='screen'
    )

    # GPS Subscriber Node
    gps_subscriber_node = Node(
        package='mobile_robot',
        executable='gps_subscriber',
        output='screen'
    )

    # Rover Control Node
    rover_control_node = Node(
        package='mobile_robot',
        executable='rover_control_node',
        output='screen'
    )

    # Obstacle Avoidance Node (New)
    obstacle_avoidance_node = Node(
        package='mobile_robot',
        executable='obstacle_avoidance',
        output='screen'
    )

    # RViz2 for visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(package_share_directory, "rviz", "rover.rviz")],
        output="screen"
    )

    # Create LaunchDescription and add all nodes
    ld = LaunchDescription()
    ld.add_action(gazebo_launch)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_node)
    ld.add_action(coordinate_input_node)
    ld.add_action(gps_subscriber_node)
    ld.add_action(rover_control_node)
    ld.add_action(obstacle_avoidance_node)
    ld.add_action(rviz_node)

    return ld
