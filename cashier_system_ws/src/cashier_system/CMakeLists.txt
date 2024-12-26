cmake_minimum_required(VERSION 3.5)
project(cashier_system)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)

# Custom message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bill.msg"
  DEPENDENCIES builtin_interfaces  # Ensure builtin_interfaces is listed here
)

# Ensure messages are included in the build
set(ROSIDL_TYPESUPPORT_TARGET "rosidl_typesupport_cpp")
rosidl_get_typesupport_target(cashier_system_typesupport_target
  ${PROJECT_NAME} ${ROSIDL_TYPESUPPORT_TARGET})

# Add executables
add_executable(bill_publisher src/bill_publisher.cpp)
ament_target_dependencies(bill_publisher rclcpp std_msgs builtin_interfaces std_srvs)
target_link_libraries(bill_publisher ${cashier_system_typesupport_target})

add_executable(inventory_manager src/inventory_manager.cpp)
ament_target_dependencies(inventory_manager rclcpp std_msgs builtin_interfaces std_srvs)
target_link_libraries(inventory_manager ${cashier_system_typesupport_target})

add_executable(last_bill_printer src/last_bill_printer.cpp)
ament_target_dependencies(last_bill_printer rclcpp std_msgs builtin_interfaces std_srvs)
target_link_libraries(last_bill_printer ${cashier_system_typesupport_target})

# Install executables
install(TARGETS
  bill_publisher
  inventory_manager
  last_bill_printer
  DESTINATION lib/${PROJECT_NAME}
)

# Install other files (only if the directories exist)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/msg)
  install(DIRECTORY msg DESTINATION share/${PROJECT_NAME})
endif()

# Enable testing if needed
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Add tests here
endif()

# Include the generated files
rosidl_target_interfaces(bill_publisher
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(inventory_manager
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(last_bill_printer
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Export package
ament_package()
