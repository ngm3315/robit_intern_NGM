cmake_minimum_required(VERSION 3.19)
project(qt_project LANGUAGES CXX)

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Widgets Network)

qt_standard_project_setup()

qt_add_executable(qt_project
    WIN32 MACOSX_BUNDLE
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
)

target_link_libraries(qt_project
    PRIVATE
        Qt::Core
        Qt::Widgets
        Qt::Network   # ← 추가
)
