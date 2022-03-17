
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

#include "behavior_trees/Turn.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/Twist.h"

#include "ros/ros.h"

namespace behavior_trees
{

Turn::Turn(const std::string& name)
: BT::ActionNodeBase(name, {})
{
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);
}

void
Turn::halt()
{
  ROS_INFO("Turn halt");
}

BT::NodeStatus
Turn::tick()
{
  geometry_msgs::Twist cmd;

  cmd.linear.x = 0.0;
  cmd.angular.z = angspeed_;
  vel_pub_.publish(cmd);
  return BT::NodeStatus::RUNNING;
}

}  // namespace behavior_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behavior_trees::Turn>("Turn");
}
