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

#ifndef BEHAVIOR_TREES_FOLLOWPERSON_H
#define BEHAVIOR_TREES_FOLLOWPERSON_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "visual_bh/Pos_person.h"
#include "br2_tracking/PIDController.h"
#include "geometry_msgs/Twist.h"
#include <string>

#include "ros/ros.h"

namespace behavior_trees
{

class FollowPerson : public BT::ActionNodeBase
{
  public:
    explicit FollowPerson(const std::string& name);

    void halt();

    BT::NodeStatus tick();

  private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    geometry_msgs::Twist vel_msgs_;
    visual_bh::Pos_person pos_person;
    br2_tracking::PIDController* pos_pid_;
    br2_tracking::PIDController* angle_pid_;
    double dist_;
    double y_;
};

}  // namespace behavior_trees

#endif  // BEHAVIOR_TREES_FOLLOWPERSON_H
