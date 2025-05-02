cmake_minimum_required(VERSION 3.8)
project(twist_filter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(COMMON_SOURCES
  src/component_filters.cpp
  src/filter_twist.cpp
)

add_executable(twist_filter_node 
  src/twist_filter_node.cpp
  ${COMMON_SOURCES}
)

ament_target_dependencies(twist_filter_node
  rclcpp
  geometry_msgs
  rcl_interfaces
)

target_link_libraries(twist_filter_node
  nlohmann_json::nlohmann_json
)

install(TARGETS
  twist_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()