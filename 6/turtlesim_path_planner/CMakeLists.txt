cmake_minimum_required(VERSION 3.8)
project(turtlesim_path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(turtle src/turtle.cpp)
add_executable(turtle_pure_pursuit src/turtle_pure_pursuit.cpp)

ament_target_dependencies(
  turtle_pure_pursuit
  rclcpp 
  turtlesim
  geometry_msgs
  std_srvs
)

ament_target_dependencies(
  turtle 
  rclcpp 
  turtlesim
  geometry_msgs
  std_srvs
)

install(TARGETS 
  turtle
  turtle_pure_pursuit
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
