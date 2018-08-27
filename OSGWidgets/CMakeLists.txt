cmake_minimum_required(VERSION 3.12)
project(OSGWidgets)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5Core CONFIG REQUIRED)
find_package(Qt5Gui CONFIG REQUIRED)

set(OSGWidgets_SRCS
        OSGWidget.cpp OSGWidget.h)
add_library(OSGWidgets SHARED ${OSGWidgets_SRCS})
target_link_libraries(OSGWidgets Qt5::Widgets)

include_directories(../Common)
link_directories(../cmake-build-release/Common)
target_link_libraries(OSGWidgets Common)