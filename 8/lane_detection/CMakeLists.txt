cmake_minimum_required(VERSION 3.8)
project(lane_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(lane src/lane.cpp src/edge_detection.cpp)
add_executable(way src/way.cpp src/edge_detection.cpp)

ament_target_dependencies(
  lane
  rclcpp 
  sensor_msgs 
  cv_bridge 
  OpenCV
  carla_msgs)

  ament_target_dependencies(
  way
  rclcpp 
  sensor_msgs 
  cv_bridge 
  OpenCV
  carla_msgs)



install(TARGETS 
  lane 
  way
  DESTINATION lib/${PROJECT_NAME})

ament_package()
