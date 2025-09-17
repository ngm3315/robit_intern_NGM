cmake_minimum_required(VERSION 3.8)
project(dual4_suscriber)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(subscriber_node src/subscriber.cpp)
ament_target_dependencies(subscriber_node
  rclcpp geometry_msgs nav_msgs tf2 tf2_ros)

install(TARGETS subscriber_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include DESTINATION include)

ament_package()
