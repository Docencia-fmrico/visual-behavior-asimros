cmake_minimum_required(VERSION 2.8.3)
project(visual-behaviour-asimros)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  sensor_msgs
  std_msgs
  tf
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  darknet_ros_msgs
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
CATKIN_DEPENDS   
  roscpp
  sensor_msgs
  std_msgs
  tf
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}
  src/ball_perceiver/ImageConverter.cpp
)

add_executable(hsv_tuner_node src/hsv_tuner_node.cpp)
target_link_libraries(hsv_tuner_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${OpenCV_LIBRARIES})

set(ROSLINT_CPP_OPTS "--filter=-runtime/printf")

roslint_cpp(
  src/hsv_tuner_node.cpp
  include/ball_perceiver/ImageConverter.h
  src/ball_perceiver/ImageConverter.cpp
)

