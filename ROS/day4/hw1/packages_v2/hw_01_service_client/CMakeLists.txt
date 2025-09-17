cmake_minimum_required(VERSION 3.16)
project(hw_01_service_client)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_executable(service_client src/service_client.cpp)
ament_target_dependencies(service_client rclcpp lifecycle_msgs)

install(TARGETS service_client
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
