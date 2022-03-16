cmake_minimum_required(VERSION 2.8.3)
project(visual_bh)

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
 src/visual_bh/Tfperson.cpp
)

add_executable(node_tf_person src/node_tf_person.cpp)
target_link_libraries(node_tf_person ${PROJECT_NAME} ${catkin_LIBRARIES})

roslint_cpp(
  src/node_tf_person.cpp
  src/visual_bh/Tfperson.cpp
  include/visual_bh/Tfperson.h
)
