cmake_minimum_required(VERSION 2.8.3)
project(visual-behavior-asimros)

# set(CMAKE_BUILD_TYPE Debug)

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
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs behaviortree_cpp_v3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
)

add_library(asr_find_ball_bt_node SHARED src/behavior_trees/FindBall.cpp)
add_library(asr_find_person_bt_node SHARED src/behavior_trees/FindPerson.cpp)
add_library(asr_turn_bt_node SHARED src/behavior_trees/Turn.cpp)
add_library(asr_follow_point_bt_node SHARED src/behavior_trees/FollowPoint.cpp)
list(APPEND plugin_libs
  asr_find_ball_bt_node
  asr_find_person_bt_node
  asr_turn_bt_node
  asr_follow_point_bt_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(nodo_behavior_tree_ball
  src/nodo_behavior_tree_ball.cpp
)
add_executable(nodo_behavior_tree_person
  src/nodo_behavior_tree_person.cpp
)
target_link_libraries(nodo_behavior_tree_ball ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})
target_link_libraries(nodo_behavior_tree_person ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})

roslint_cpp(
  src/nodo_behavior_tree_ball.cpp
  include/behavior_trees/FindBall.h src/behavior_trees/FindBall.cpp
  include/behavior_trees/FollowPoint.h src/behavior_trees/FollowPoint.cpp
  include/behavior_trees/Turn.h src/behavior_trees/Turn.cpp
  src/nodo_behavior_tree_person.cpp
  include/behavior_trees/FindPerson.h src/behavior_trees/FindPerson.cpp
  include/behavior_trees/FollowPoint.h src/behavior_trees/FollowPoint.cpp
  include/behavior_trees/Turn.h src/behavior_trees/Turn.cpp
)
