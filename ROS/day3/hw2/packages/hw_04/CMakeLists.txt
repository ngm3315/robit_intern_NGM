cmake_minimum_required(VERSION 3.16)
project(hw_04)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets)

# include
include_directories(
  include
)

# Qt 자동 처리
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

# 실행 파일
add_executable(hw_04_node
  src/main.cpp
  src/main_window.cpp
  src/qnode.cpp
  ui/mainwindow.ui
  resources/images.qrc
  include/hw_04/main_window.hpp
  include/hw_04/qnode.hpp
)

target_link_libraries(hw_04_node
  Qt6::Widgets
)

ament_target_dependencies(hw_04_node
  rclcpp
)

# 실행 파일 설치
install(TARGETS hw_04_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 런치 파일 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# config 파라미터 파일 설치
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

