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

#ifndef BEHAVIOR_TREES_FINDPERSON_H
#define BEHAVIOR_TREES_FINDPERSON_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "visual_bh/Pos_person.h"
#include <string>

#include "ros/ros.h"

namespace behavior_trees
{

class FindPerson : public BT::ActionNodeBase
{
  public:
    explicit FindPerson(const std::string& name);

    void halt();

    BT::NodeStatus tick();


  private:
    double dist_;
    double y_;
    visual_bh::Pos_person pos_person;
};

}  // namespace behavior_trees

#endif  // BEHAVIOR_TREES_FINDPERSON_H
