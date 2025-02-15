cmake_minimum_required(VERSION 3.8)
project(mobile_robot)

# Set compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(GeographicLib REQUIRED)      # For geospatial calculations
find_package(std_srvs REQUIRED)           # For service interfaces
find_package(controller_manager REQUIRED) # For controller management

# Add executables with updated dependencies
add_executable(gps_subscriber src/gps_subscriber.cpp)
ament_target_dependencies(gps_subscriber
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
  nav_msgs
)

add_executable(rover_control_node src/rover_control_node.cpp)
target_link_libraries(rover_control_node ${GeographicLib_LIBRARIES})
ament_target_dependencies(rover_control_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
)

add_executable(coordinate_input_node src/coordinate_input_node.cpp)
ament_target_dependencies(coordinate_input_node 
  rclcpp 
  std_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
)

add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)
ament_target_dependencies(obstacle_avoidance
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  visualization_msgs
)

# Install targets
install(TARGETS
  coordinate_input_node
  gps_subscriber
  rover_control_node
  obstacle_avoidance
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install models and worlds
install(DIRECTORY
  model/
  DESTINATION share/${PROJECT_NAME}/model
)

# Install RViz configurations
install(DIRECTORY
  rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()