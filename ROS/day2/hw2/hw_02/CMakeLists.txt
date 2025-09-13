cmake_minimum_required(VERSION 3.16)
project(hw_02)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

find_package(Qt6 COMPONENTS Widgets QUIET)
if(Qt6_FOUND)
  set(QT_LIB Qt6::Widgets)
else()
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
  set(QT_LIB Qt5::Widgets)
endif()

set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

add_executable(hw_02_node
  src/main.cpp
  src/main_window.cpp
  src/qnode.cpp
  ui/mainwindow.ui
  resources/images.qrc
  include/hw_02/main_window.hpp
  include/hw_02/qnode.hpp
)

target_include_directories(hw_02_node PUBLIC include)

ament_target_dependencies(hw_02_node
  rclcpp
  geometry_msgs
  turtlesim
)

target_link_libraries(hw_02_node ${QT_LIB})

install(TARGETS hw_02_node DESTINATION lib/${PROJECT_NAME})
ament_package()
