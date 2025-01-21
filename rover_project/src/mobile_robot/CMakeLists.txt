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
find_package(geometry_msgs REQUIRED)  # Added geometry_msgs dependency
find_package(nav_msgs REQUIRED)       # Added nav_msgs dependency (if needed for rover control)

# Add executables
add_executable(gps_subscriber src/gps_subscriber.cpp)
ament_target_dependencies(gps_subscriber
  rclcpp
  sensor_msgs
)

add_executable(rover_control_node src/rover_control_node.cpp) 
ament_target_dependencies(rover_control_node
  rclcpp
  std_msgs
  geometry_msgs  # Add geometry_msgs if using geometry-related messages for rover control
  nav_msgs       # Add nav_msgs if using navigation messages (optional based on rover control needs)
)

add_executable(coordinate_input_node src/coordinate_input_node.cpp)
ament_target_dependencies(coordinate_input_node 
  rclcpp 
  std_msgs
)

# Install executables
install(TARGETS
  coordinate_input_node
  gps_subscriber
  rover_control_node  # Added the new node for installation
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories for launch and model files
install(DIRECTORY launch model
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch model
        DESTINATION install/${PROJECT_NAME})

# Run linter if testing is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  # Comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # Comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
