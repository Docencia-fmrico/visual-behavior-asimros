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

#include "visual_behaviour/Data_tf.h"

#define OBJ_PUB_RATE  10.0

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_tf_person");

  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::Image> image_depth_sub(nh, "/camera/depth/image_raw", 1);
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub(nh, "/darknet_ros/bounding_boxes", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
  message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx(MySyncPolicy_bbx(10), image_depth_sub, bbx_sub);

  Data_tf data_tf(false);
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped bf2person;
  ros::Rate loop_rate(20);

  sync_bbx.registerCallback(boost::bind(&data_tf.callback_bbx, _1, _2));
  
  while(ros::ok)
  {
    if (data_tf.get_update())
    {
      try
      {
        bf2person = data_tf.generate_tf();
        bf2person.header.stamp = ros::Time::now();
        br.sendTransform(bf2person);
      }
      catch(tf2::TransformException &exception)
      {
        ROS_ERROR("%s", exception.what());
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}