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

#include "behavior_trees/FollowPerson.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace behavior_trees
{

FollowPerson::FollowPerson(const std::string& name)
: BT::ActionNodeBase(name, {})
{
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);
    pos_pid_ = new br2_tracking::PIDController(0.0, 2.0, 0, 0.4);
    angle_pid_ = new br2_tracking::PIDController(0.0, 50.0, 0, 0.4);
}

void
FollowPerson::halt()
{
  ROS_INFO("FollowPerson halt");
}

BT::NodeStatus
FollowPerson::tick()
{
  dist_ = pos_person.x();
  y_ = pos_person.y();

  if (!std::isnan(dist_) && !(dist_ <= 0.0) && ((ros::Time::now()-pos_person.getTime()).toSec()) < 1.0)
  {
    vel_msgs_.linear.x = pos_pid_->get_output(dist_ - 1.0);
    vel_msgs_.angular.z = angle_pid_->get_output(340 - y_);
    vel_pub_.publish(vel_msgs_);
    return BT::NodeStatus::RUNNING;
  }
  else
  {
    vel_msgs_.linear.x = 0.0;
    vel_msgs_.angular.z = 0.0;
    vel_pub_.publish(vel_msgs_);
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace behavior_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behavior_trees::FollowPerson>("FollowPerson");
}
