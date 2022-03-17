// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include "behavior_trees/FindBall.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros/ros.h"

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Twist.h"
#include "tf2/convert.h"
#include "br2_tracking/PIDController.h"

namespace behavior_trees
{

FindBall::FindBall(const std::string& name)
: BT::ActionNodeBase(name, {})
{
}


void
FindBall::halt()
{
  ROS_INFO("FindBall halt");
}

BT::NodeStatus
FindBall::tick()
{
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);

  if (buffer.canTransform("base_footprint", "ball/0", ros::Time(0), ros::Duration(1.0), &error))
  {
    ROS_INFO("He visto la bola");
    return BT::NodeStatus::SUCCESS;
  }
  ROS_INFO("No he visto la bola");
  return BT::NodeStatus::FAILURE;
}

}  // namespace behavior_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behavior_trees::FindBall>("FindBall");
}
