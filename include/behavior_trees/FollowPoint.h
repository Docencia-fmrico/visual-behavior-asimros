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

#ifndef BEHAVIOR_TREES_FOLLOWPOINT_H
#define BEHAVIOR_TREES_FOLLOWPOINT_H

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

class FollowPoint : public BT::ActionNodeBase
{
  public:
    explicit FollowPoint(const std::string& name);

    void halt();

    BT::NodeStatus tick();

  private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    geometry_msgs::Twist vel_msgs_;
    geometry_msgs::TransformStamped bf2ball_msg;
    tf2::Stamped<tf2::Transform> bf2ball;
    std::string error;
    br2_tracking::PIDController* pos_pid_;
    br2_tracking::PIDController* angle_pid_;
};

}  // namespace behavior_trees

#endif  // BEHAVIOR_TREES_FOLLOWPOINT_H
