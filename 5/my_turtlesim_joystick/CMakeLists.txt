cmake_minimum_required(VERSION 3.8)
project(my_turtlesim_joystick)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(joystick src/joystick.cpp)
ament_target_dependencies(
  joystick 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  turtlesim
  std_srvs
)

install(TARGETS 
  joystick
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
