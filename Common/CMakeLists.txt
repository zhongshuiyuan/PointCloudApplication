cmake_minimum_required(VERSION 3.12)

project(Common)

set(CMAKE_CXX_STANDARD 14)

set(Common_SRCS
        tracer.h
        VectorMapSingleton.cpp VectorMapSingleton.h DataStructure.h common.h)

ADD_LIBRARY(Common SHARED ${Common_SRCS})