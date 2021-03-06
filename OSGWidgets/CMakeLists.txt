cmake_minimum_required(VERSION 3.12)
project(OSGWidgets)
add_subdirectory(bezier_curve)

set(CMAKE_CXX_STANDARD 14)

#Qt related
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5Core CONFIG REQUIRED)
find_package(Qt5Gui CONFIG REQUIRED)
find_package(Qt5OpenGL REQUIRED)
set(OSGWidgets_SRCS
        BezierCurve.cpp
        OSGWidget.cpp
        LineEditor.cpp
        TraceEditor.cpp
        SelectEditor.cpp
        VMapDrawable.cpp
        PositionTransformer.cpp
        )
add_library(OSGWidgets SHARED ${OSGWidgets_SRCS})
target_link_libraries(OSGWidgets Qt5::Widgets Qt5::Core)

#other module
include_directories(../Common
        ./bezier_curve
        )
#link_directories(../cmake-build-release/Common)
target_link_libraries(OSGWidgets
        Common
        Bezier
        )


#osgearth
set(OSGEarth_LIBRARIES "libosgEarth.so;libosgEarthAnnotation.so;libosgEarthFeatures.so;libosgEarthQt.so;libosgEarthSymbology.so;libosgEarthUtil.so")

#osg & pcl & eigen
include_directories(
        ${OPENSCENEGRAPH_INCLUDE_DIRS}
        ${OSGEARTH_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

find_package(OpenSceneGraph 3.2 REQUIRED COMPONENTS osgGA osgViewer osgDB osgUtil osg osgQt osgFX)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
target_link_libraries(OSGWidgets
        ${OPENSCENEGRAPH_LIBRARIES}
        ${OSGEarth_LIBRARIES}
        ${PCL_LIBRARIES}
        )