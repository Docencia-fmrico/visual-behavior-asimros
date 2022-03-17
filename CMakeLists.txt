cmake_minimum_required(VERSION 2.8.3)
project(visual-behavior-asimros)
set(CMAKE_CXX_FLAGS "-std=c++17")

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  behaviortree_cpp_v3
  roslint
  roslint
  sensor_msgs
  tf
  cv_bridge
  image_transport
  darknet_ros_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  behaviortree_cpp_v3
  sensor_msgs
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
  ${ZMQ_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_library(asr_find_ball_node SHARED src/behavior_trees/FindBall.cpp)
add_library(asr_find_person_node SHARED src/behavior_trees/FindPerson.cpp)
add_library(asr_turn_node SHARED src/behavior_trees/Turn.cpp)
add_library(asr_follow_point_node SHARED src/behavior_trees/FollowPoint.cpp)
list(APPEND plugin_libs
  asr_find_ball_node
  asr_find_person_node
  asr_turn_node
  asr_follow_point_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${PROJECT_NAME} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_library(${PROJECT_NAME}(
  src/visual_bh/Pos_person.cpp
  src/br2_tracking/PIDController.cpp
  src/ball_positioning/image_filter.cpp
  src/ball_positioning/cloud_filter.cpp
  src/ball_positioning/tf_ball.cpp
)

add_executable(image_filter_node src/image_filter_node.cpp)
target_link_libraries(image_filter_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(cloud_filter_node src/cloud_filter_node.cpp)
target_link_libraries(cloud_filter_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(tf_ball_node src/tf_ball_node.cpp)
target_link_libraries(tf_ball_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(nodo_behavior_tree_ball src/nodo_behavior_tree_ball.cpp)
target_link_libraries(nodo_behavior_tree_ball ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})
add_executable(nodo_behavior_tree_person src/nodo_behavior_tree_person.cpp)
target_link_libraries(nodo_behavior_tree_person ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})

add_executable(follow_example src/follow_example.cpp)
target_link_libraries(follow_example ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(node_follow_person src/node_follow_person.cpp)
target_link_libraries(node_follow_person ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

roslint_cpp(
  src/nodo_behavior_tree_ball.cpp
  include/behavior_trees/FindBall.h src/behavior_trees/FindBall.cpp
  include/behavior_trees/FollowPoint.h src/behavior_trees/FollowPoint.cpp
  include/behavior_trees/Turn.h src/behavior_trees/Turn.cpp
  src/nodo_behavior_tree_person.cpp
  include/behavior_trees/FindPerson.h src/behavior_trees/FindPerson.cpp
  include/behavior_trees/FollowPoint.h src/behavior_trees/FollowPoint.cpp
  include/behavior_trees/Turn.h src/behavior_trees/Turn.cpp
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
  src/node_follow_person.cpp
  src/visual_bh/Pos_person.cpp
  include/visual_bh/Pos_person.h
)
