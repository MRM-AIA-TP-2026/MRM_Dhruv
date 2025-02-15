cmake_minimum_required(VERSION 3.8)
project(mobile_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)  # Required for IMU quaternion conversion
find_package(tf2_ros REQUIRED)  # Required for TF transformations
find_package(visualization_msgs REQUIRED)  # Required for RViz visualization
find_package(rviz2 REQUIRED)  # Required for RViz integration

# Add executables
add_executable(gps_subscriber src/gps_subscriber.cpp)
ament_target_dependencies(gps_subscriber
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(rover_control_node src/rover_control_node.cpp) 
ament_target_dependencies(rover_control_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_geometry_msgs
)

add_executable(coordinate_input_node src/coordinate_input_node.cpp)
ament_target_dependencies(coordinate_input_node 
  rclcpp 
  std_msgs
  sensor_msgs
)

# New: Obstacle Avoidance Node
add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)
ament_target_dependencies(obstacle_avoidance
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  tf2_ros
)

# Install executables
install(TARGETS
  coordinate_input_node
  gps_subscriber
  rover_control_node
  obstacle_avoidance  # New obstacle avoidance node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories for launch, model, and rviz files
install(DIRECTORY
  launch/
  rviz/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  model/
  DESTINATION share/${PROJECT_NAME}/model
)

install(DIRECTORY
  rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
