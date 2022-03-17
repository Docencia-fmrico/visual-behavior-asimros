#include "visual_bh/Tfperson.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "ros/ros.h"

#include "geometry_msgs/Twist.h"

#define OBJ_PUB_RATE  10.0

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_tf_person");

  visual_bh::Tfperson tf_person(false);
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped bf2person;
  ros::Rate loop_rate(10);
  
  while(ros::ok)
  {
    ROS_INFO("holis");
    if (tf_person.get_update())
    {
      try
      {
        bf2person = tf_person.generate_tf();
        bf2person.header.stamp = ros::Time::now();
        br.sendTransform(bf2person);
      }
      catch(tf2::TransformException &exception)
      {
        ROS_ERROR("%s", exception.what());
      }
    }
    ROS_INFO("holis");
    ros::spin();
    loop_rate.sleep();
  }

  return 0;
}