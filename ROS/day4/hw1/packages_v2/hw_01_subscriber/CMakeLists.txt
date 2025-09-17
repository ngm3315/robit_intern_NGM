cmake_minimum_required(VERSION 3.16)
project(hw_01_subscriber)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(subscriber_node src/subscriber.cpp)
ament_target_dependencies(subscriber_node rclcpp rclcpp_lifecycle std_msgs)

install(TARGETS subscriber_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
