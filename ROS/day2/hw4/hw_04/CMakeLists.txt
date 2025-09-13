cmake_minimum_required(VERSION 3.16)
project(hw_04)

set(CMAKE_CXX_STANDARD 17)
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

set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)
include_directories(include)

add_executable(hw_04_node
  src/main.cpp
  src/main_window.cpp
  src/qnode.cpp
  resources/images.qrc
  ui/mainwindow.ui
  include/hw_04/main_window.hpp
  include/hw_04/qnode.hpp
)

target_link_libraries(hw_04_node ${QT_LIB})

ament_target_dependencies(hw_04_node
  rclcpp
  custom_interfaces
)

install(TARGETS hw_04_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
