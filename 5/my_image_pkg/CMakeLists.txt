cmake_minimum_required(VERSION 3.8)
project(my_image_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${cv_bridge_INCLUDE_DIRS})

add_executable(image_publisher src/image_publisher.cpp)
ament_target_dependencies(image_publisher rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(image_subscriber src/image_subscriber.cpp)
ament_target_dependencies(image_subscriber rclcpp sensor_msgs cv_bridge OpenCV)

install(TARGETS 
  image_publisher
  image_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
