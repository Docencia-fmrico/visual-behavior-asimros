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

#include "behavior_trees/FindPerson.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace behavior_trees
{

FindPerson::FindPerson(const std::string& name)
: BT::ActionNodeBase(name, {})
{
}

void
FindPerson::halt()
{
  ROS_INFO("FindPerson halt");
}

BT::NodeStatus
FindPerson::tick()
{
  dist_ = pos_person.x() / 100;
  y_ = pos_person.y();

  if (!std::isnan(dist_) && !(dist_ <= 0.0) && ((ros::Time::now()-pos_person.getTime()).toSec()) < 1.0)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace behavior_trees

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behavior_trees::FindPerson>("FindPerson");
}
