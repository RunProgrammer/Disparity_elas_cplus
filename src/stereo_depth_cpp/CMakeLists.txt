cmake_minimum_required(VERSION 3.8)
project(stereo_depth_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
# target_link_libraries(stereobm_node Threads::Threads)

include_directories(include)

add_executable(stereobm_node
  src/main.cpp
  src/stereobm_node.cpp
)

ament_target_dependencies(stereobm_node
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
  OpenCV
)

install(TARGETS stereobm_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
