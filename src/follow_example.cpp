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

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Twist.h"
#include "tf2/convert.h"

#include "br2_tracking/PIDController.hpp"

#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_sub");
  ros::NodeHandle n;

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);

  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);
  br2_tracking::PIDController pos_pid(1.0, 10.0, 0, 0.5);
  br2_tracking::PIDController neg_pid(1.0, 0.0, 0, -0.5);
  geometry_msgs::Twist vel_msgs;

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    geometry_msgs::TransformStamped bf2ball_msg;
    tf2::Stamped<tf2::Transform> bf2ball;
    std::string error;
    if (buffer.canTransform("base_footprint", "ball/0", ros::Time(0), ros::Duration(0.2), &error))
    {
      bf2ball_msg = buffer.lookupTransform("base_footprint", "ball/0", ros::Time(0));

      tf2::fromMsg(bf2ball_msg, bf2ball);

      double dist = bf2ball.getOrigin().length();
      double angle = atan2(bf2ball.getOrigin().y(), bf2ball.getOrigin().x());
      vel_msgs.linear.x = pos_pid.get_output(dist);
      vel_msgs.angular.z = angle;
      ROS_INFO("%f", angle);

    }
    else
    {
      vel_msgs.linear.x = 0.0;
      vel_msgs.angular.z = 0.5;
      ROS_ERROR("%s", error.c_str());
    }

    vel_pub.publish(vel_msgs);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
