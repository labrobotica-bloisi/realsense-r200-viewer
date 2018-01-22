cmake_minimum_required(VERSION 2.8.3)
project(realsense_r200_viewer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package()

include_directories(
  src/
  ${catkin_INCLUDE_DIRS}
)

set(SRCS src/r200_viewer.cpp)

add_executable(realsense_r200_viewer ${SRCS})

target_link_libraries(realsense_r200_viewer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

