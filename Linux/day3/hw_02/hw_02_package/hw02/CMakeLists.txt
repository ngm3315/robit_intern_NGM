cmake_minimum_required(VERSION 3.21)
project(hw02 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTORCC OFF)

find_package(Qt6 REQUIRED COMPONENTS Widgets Gui)

qt_add_executable(hw02
    src/main.cpp
    src/mainwindow.cpp
    include/mainwindow.h
)

target_include_directories(hw02 PRIVATE include)
target_link_libraries(hw02 PRIVATE Qt6::Widgets Qt6::Gui)
