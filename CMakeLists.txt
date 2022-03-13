cmake_minimum_required(VERSION 2.8.3)
project(visual-behaviour-asimros)
set(CMAKE_CXX_FLAGS "-std=c++17")

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
  src/ball_positioning/image_filter.cpp
  src/ball_positioning/cloud_filter.cpp
  src/ball_positioning/tf_ball.cpp
  src/br2_tracking/PIDController.cpp  
)

add_executable(image_filter_node src/image_filter_node.cpp)
target_link_libraries(image_filter_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(cloud_filter_node src/cloud_filter_node.cpp)
target_link_libraries(cloud_filter_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(tf_ball_node src/tf_ball_node.cpp)
target_link_libraries(tf_ball_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(follow_example src/follow_example.cpp)
target_link_libraries(follow_example ${PROJECT_NAME} ${catkin_LIBRARIES})

roslint_cpp(
  include/br2_tracking/PIDController.hpp
  src/br2_tracking/PIDController.cpp
  include/ball_positioning/image_filter.h
  src/ball_positioning/image_filter.cpp
  include/ball_positioning/cloud_filter.h
  src/ball_positioning/cloud_filter.cpp
  include/ball_positioning/tf_ball.h
  src/ball_positioning/tf_ball.cpp
  src/image_filter_node.cpp
  src/cloud_filter_node.cpp
  src/tf_ball_node.cpp
  src/follow_example.cpp
)