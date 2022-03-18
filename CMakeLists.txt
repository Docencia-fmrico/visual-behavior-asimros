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
  pcl_conversions
  pcl_ros
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

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
add_library(asr_follow_person_node SHARED src/behavior_trees/FollowPerson.cpp)
add_library(asr_turn_node SHARED src/behavior_trees/Turn.cpp)
add_library(asr_follow_point_node SHARED src/behavior_trees/FollowPoint.cpp)
list(APPEND plugin_libs
  asr_find_ball_node
  asr_find_person_node
  asr_follow_person_node
  asr_turn_node
  asr_follow_point_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_library(${PROJECT_NAME}
  src/visual_bh/Pos_person.cpp
  src/br2_tracking/PIDController.cpp
)

add_executable(nodo_behavior_tree_ball src/nodo_behavior_tree_ball.cpp)
target_link_libraries(nodo_behavior_tree_ball ${PROJECT_NAME} ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})
add_executable(nodo_behavior_tree_person src/nodo_behavior_tree_person.cpp)
target_link_libraries(nodo_behavior_tree_person ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${ZMQ_LIBRARIES})
add_executable(nodo_behavior_tree_both src/nodo_behavior_tree_both.cpp)
target_link_libraries(nodo_behavior_tree_both ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${ZMQ_LIBRARIES})

add_executable(follow_ball_example src/follow_ball_example.cpp)
target_link_libraries(follow_ball_example ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(follow_person_example src/follow_person_example.cpp)
target_link_libraries(follow_person_example ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

roslint_cpp(
  src/nodo_behavior_tree_ball.cpp
  include/behavior_trees/FindBall.h src/behavior_trees/FindBall.cpp
  include/behavior_trees/FollowPoint.h src/behavior_trees/FollowPoint.cpp
  include/behavior_trees/Turn.h src/behavior_trees/Turn.cpp
  src/nodo_behavior_tree_person.cpp
  src/nodo_behavior_tree_both.cpp
  include/behavior_trees/FindPerson.h src/behavior_trees/FindPerson.cpp
  include/behavior_trees/FollowPerson.h src/behavior_trees/FollowPerson.cpp
  include/behavior_trees/FollowPoint.h src/behavior_trees/FollowPoint.cpp
  include/behavior_trees/Turn.h src/behavior_trees/Turn.cpp
  include/br2_tracking/PIDController.h
  src/br2_tracking/PIDController.cpp
  src/follow_ball_example.cpp
  src/follow_person_example.cpp
  src/visual_bh/Pos_person.cpp
  include/visual_bh/Pos_person.h
)
