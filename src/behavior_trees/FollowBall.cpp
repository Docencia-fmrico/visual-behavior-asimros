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
#include "behavior_trees/FollowBall.h"
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

FollowBall::FollowBall(const std::string& name)
: BT::ActionNodeBase(name, {})
{
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);
  pos_pid_ = new br2_tracking::PIDController(0.0, 2.0, 0, 0.4);
  angle_pid_ = new br2_tracking::PIDController(0.0, 1.5, 0, 1.0);
}

void
FollowBall::halt()
{
  ROS_INFO("FollowBall halt");
}

BT::NodeStatus
FollowBall::tick()
{
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);

  if (buffer.canTransform("base_footprint", "ball/0", ros::Time(0), ros::Duration(1.0), &error))
  {
    bf2ball_msg = buffer.lookupTransform("base_footprint", "ball/0", ros::Time(0));

    tf2::fromMsg(bf2ball_msg, bf2ball);

    double dist = bf2ball.getOrigin().length();
    double angle = atan2(bf2ball.getOrigin().y(), bf2ball.getOrigin().x());
    vel_msgs_.linear.x = pos_pid_->get_output(dist - 1.0);
    ROS_INFO("%f",  angle_pid_->get_output(angle));
    vel_msgs_.angular.z = angle_pid_->get_output(angle);
    vel_pub_.publish(vel_msgs_);
    return BT::NodeStatus::RUNNING;
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
  factory.registerNodeType<behavior_trees::FollowBall>("FollowBall");
}
