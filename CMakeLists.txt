cmake_minimum_required(VERSION 3.10)
project(octomap_builder)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES octo_test
#  CATKIN_DEPENDS roscpp std_msg
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(k4a 1.3.0 QUIET)
include_directories(${K4A_INCLUDE_DIRS})

set(OpenCV_DIR /usr/local/lib/cmake/opencv4/)
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS} include)

add_executable(octobuilder main.cpp)
target_sources(octobuilder PRIVATE ${PROJECT_SOURCE_DIR}/src/DataManager.cpp 
${PROJECT_SOURCE_DIR}/src/Frame.cpp
${PROJECT_SOURCE_DIR}/src/MapPoint.cpp
${PROJECT_SOURCE_DIR}/src/System_SLAM.cpp
${PROJECT_SOURCE_DIR}/src/Tracking.cpp)
target_link_libraries(octobuilder PRIVATE ${OpenCV_LIBS} k4a::k4a ${catkin_LIBRARIES})

