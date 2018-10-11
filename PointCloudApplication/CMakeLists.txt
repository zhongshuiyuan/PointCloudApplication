cmake_minimum_required(VERSION 3.12)
project(PointCloudApplication)

set(CMAKE_CXX_STANDARD 14)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Create code from a list of Qt designer ui files
#set(CMAKE_AUTOUIC ON)

# Find the QtWidgets library
find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5Core CONFIG REQUIRED)

# Populate a CMake variable with the sources
set(PointCloudApplication_SRCS
        main.cpp
        MainWindow.cpp MainWindow.h EditorDialog.cpp EditorDialog.h)
# Tell CMake to create the helloworld executable
add_executable(PointCloudApplication ${PointCloudApplication_SRCS})
# Use the Widgets module from Qt 5
target_link_libraries(PointCloudApplication Qt5::Widgets)

# Other library
include_directories(../Common)
#link_directories(../cmake-build-release/Common)
target_link_libraries(PointCloudApplication Common)

include_directories(../OSGWidgets)
#link_directories(../cmake-build-release/OSGWidgets)
target_link_libraries(PointCloudApplication OSGWidgets)

