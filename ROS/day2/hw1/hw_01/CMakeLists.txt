cmake_minimum_required(VERSION 3.16)
project(hw_01)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)

# Qt6 우선, 없으면 Qt5
find_package(Qt6 COMPONENTS Widgets QUIET)
if(Qt6_FOUND)
  set(QT_LIB Qt6::Widgets)
else()
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
  set(QT_LIB Qt5::Widgets)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

add_executable(hw_01_node
  src/main.cpp
  src/main_window.cpp
  src/qnode.cpp
  ui/MainWindow.ui
  resources/images.qrc
  include/hw_01/main_window.hpp
  include/hw_01/qnode.hpp
)
target_link_libraries(hw_01_node ${QT_LIB})
ament_target_dependencies(hw_01_node rclcpp custom_interfaces)

add_executable(publisher_node src/publisher.cpp)
ament_target_dependencies(publisher_node rclcpp custom_interfaces)

add_executable(subscriber_node src/subscriber.cpp)
ament_target_dependencies(subscriber_node rclcpp custom_interfaces)

install(TARGETS
  hw_01_node
  publisher_node
  subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
