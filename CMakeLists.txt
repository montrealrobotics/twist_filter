cmake_minimum_required(VERSION 3.8)
project(twist_filter)

# Default to C++14 for ROS2
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find the parameters generator for dynamic parameters
find_package(rosidl_typesupport_cpp REQUIRED)

# Install launch files
install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/filter_twist.py
  src/init_fir.py
  src/init_iir.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(
  DIRECTORY
    config
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclpy
  std_msgs
  geometry_msgs
)

# Package configuration
ament_package()