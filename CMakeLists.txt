cmake_minimum_required(VERSION 3.10)
project(OCTOBUILDER)

find_package(k4a 1.3.0 QUIET)
include_directories(${K4A_INCLUDE_DIRS})

set(OpenCV_DIR /usr/local/lib/cmake/opencv4/)
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(octobuilder main.cpp)
target_link_libraries(octobuilder PRIVATE ${OpenCV_LIBS} k4a::k4a )

