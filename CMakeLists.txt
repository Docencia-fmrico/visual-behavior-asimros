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
  darknet_ros_msgs
  geometry_msgs
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
)

link_directories(
	${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}
  src/visual_bh/Pos_person.cpp
)

add_executable(node_follow_person src/node_follow_person.cpp)
target_link_libraries(node_follow_person ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

roslint_cpp(
  src/node_follow_person.cpp
  src/visual_bh/Pos_person.cpp
  include/visual_bh/Pos_person.h
)
