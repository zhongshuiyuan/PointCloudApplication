cmake_minimum_required(VERSION 3.12)
project(BezierCurve)

set(CMAKE_CXX_STANDARD 14)

set(Bezier_SRCS
        bezier.cpp
        bezier_adapter.cpp
        )
add_library(Bezier SHARED ${Bezier_SRCS})