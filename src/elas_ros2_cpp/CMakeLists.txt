cmake_minimum_required(VERSION 3.8)
project(elas_ros2_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  third_party/libelas   # adjust based on actual libelas folder layout
)

file(GLOB LIBELAS_SOURCES third_party/libelas/*.cpp)

add_executable(elas_node
  src/main.cpp
  src/elas_node.cpp
  ${LIBELAS_SOURCES}
)

ament_target_dependencies(elas_node
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
  OpenCV
)

install(TARGETS elas_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
