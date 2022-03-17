
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "br2_tracking/PIDController.hpp"

#include "visual_bh/Pos_person.h"

#include "ros/ros.h"

#include "geometry_msgs/Twist.h"

#define OBJ_PUB_RATE  10.0

double angular_velocity(double y)
{
  double middle = 340;

  if(y > middle) {
    if(y - middle < 50){
      return -0.1; 
    } else{
      return -0.3;  
    }
  } else{
    if(middle - y < 50){
      return 0.1;
    } else{
      return 0.3;
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_pos_person");

  ros::NodeHandle n;
  geometry_msgs::Twist cmd;
 
  visual_bh::Pos_person pos_person;
  ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  ros::Rate loop_rate(10);  
  br2_tracking::PIDController vel_pid(0.0, 2.0, 0, 0.4);
  br2_tracking::PIDController angle_pid(290, 390, 0.4, -0.4);
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;

  while(ros::ok)
  {    
    //if comprobar si time es reciente
    double dist = pos_person.x();
    double y = pos_person.y();
    if(!std::isnan(dist) && !(dist <= 0.0) && ((ros::Time::now()-pos_person.getTime()).toSec()) < 1.0)
    {
      ROS_INFO("%f, %f", dist, y);
      cmd.linear.x = vel_pid.get_output(dist - 1.0);
      cmd.angular.z = angular_velocity(y);
    }
    else {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.3;
    }

    pub_vel.publish(cmd);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}