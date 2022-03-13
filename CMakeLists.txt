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
  src/br2_tracking/PIDController.cpp  
)

add_executable(image_filter src/image_filter.cpp)
target_link_libraries(image_filter ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(cloud_filter src/cloud_filter.cpp)
target_link_libraries(cloud_filter ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(tf_ball src/tf_ball.cpp)
target_link_libraries(tf_ball ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(follow_example src/follow_example.cpp)
target_link_libraries(follow_example ${PROJECT_NAME} ${catkin_LIBRARIES})

roslint_cpp(
  include/br2_tracking/PIDController.hpp
  src/br2_tracking/PIDController.cpp
  src/image_filter.cpp
  src/cloud_filter.cpp
  src/tf_ball.cpp
  src/follow_example.cpp
)