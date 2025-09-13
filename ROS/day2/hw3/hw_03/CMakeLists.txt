cmake_minimum_required(VERSION 3.16)
project(hw_03)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Qt6 COMPONENTS Widgets QUIET)
if(Qt6_FOUND)
  set(QT_LIB Qt6::Widgets)
else()
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
  set(QT_LIB Qt5::Widgets)
endif()

set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

add_executable(hw_03_node
  src/main.cpp
  src/main_window.cpp
  src/qnode.cpp
  ui/mainwindow.ui
  resources/images.qrc
  include/hw_03/main_window.hpp
  include/hw_03/qnode.hpp
)

target_include_directories(hw_03_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(hw_03_node rclcpp std_msgs)
target_link_libraries(hw_03_node ${QT_LIB})

install(TARGETS hw_03_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()
