cmake_minimum_required(VERSION 3.16)
project(hw_01)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(hw_01_node src/hw_01.cpp)
ament_target_dependencies(hw_01_node rclcpp geometry_msgs turtlesim)

install(TARGETS hw_01_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
