import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Robot and file parameters
    robotXacroname = 'rover'
    namePackage = 'mobile_robot'
    modelFileRelativePath = 'model/rover.xacro'
    worldFileRelativePath = 'model/world.world'

    # Construct file paths
    pathModelFile = os.path.join(get_package_share_directory('mobile_robot'), 'model', 'rover.xacro')

    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)

    # Process the Xacro file to generate robot description
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # Include Gazebo launch file directly (assuming 'gazebo_ros' package and its launch file)
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')  # Adjust package if needed
    )
    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch,
        launch_arguments=[('world', pathWorldFile)]  # Fixed the syntax here
    )

    # Nodes for spawning the model and publishing the robot state
    spawnModelNode = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robotXacroname],
        output='screen'
    )
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription, 'use_sim_time': True}]
    )

    # Add your coordinate input node that publishes target bearing
    coordinateInputNode = Node(
        package='mobile_robot',
        executable='coordinate_input_node',
        output='screen'
    )

    # Add your GPS subscriber node
    gpsSubscriberNode = Node(
        package='mobile_robot',
        executable='gps_subscriber',
        output='screen'
    )

    # Add your rover control node (if you have one to move the rover based on bearing)
    roverControlNode = Node(
        package='mobile_robot',
        executable='rover_control_node',  # Replace with your control node executable name
        output='screen'
    )

    print(f"Path to model file: {pathModelFile}")
    print(f"Path to world file: {pathWorldFile}")

    # Create and populate launch description
    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(spawnModelNode)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(coordinateInputNode)
    launchDescriptionObject.add_action(gpsSubscriberNode)
    launchDescriptionObject.add_action(roverControlNode)

    return launchDescriptionObject
