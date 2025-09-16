cmake_minimum_required(VERSION 3.16)
project(hw_01_publisher)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(publisher_node src/publisher.cpp)
ament_target_dependencies(publisher_node rclcpp rclcpp_lifecycle std_msgs)

install(TARGETS publisher_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
