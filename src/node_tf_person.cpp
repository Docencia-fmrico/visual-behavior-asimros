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

#define OBJ_PUB_RATE  10.0

geometry_msgs::TransformStamped generate_tf(cv::Point2d p)
{
  tf2::Stamped<tf2::Transform> object;
  object.frame_id_ = "base_footprint";
  object.stamp_ = ros::Time::now();

  object.setOrigin(tf2::Vector3(p.x, p.y, 0.0));

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  object.setRotation(q);

  geometry_msgs::TransformStamped object_msg = tf2::toMsg(object);
  object_msg.child_frame_id = "object";

  return object_msg;
}

void callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
{
  cv_bridge::CvImagePtr img_ptr_depth;
  cv::Point2d p(0.0f, 0.0f);
  tf2_ros::TransformBroadcaster br;

  try{
      img_ptr_depth = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  
  for(const auto & box : boxes->bounding_boxes) {
    int px = (box.xmax + box.xmin) / 2;
    int py = (box.ymax + box.ymin) / 2;

    p.x = img_ptr_depth->image.at<float> (cv::Point(px, py));
    p.y = px;

    std::cerr << box.Class << " at x (dist): " << p.x << ", y (x): " << p.y << std::endl;

    if(std::isnan(p.x))
    {
      return;
    }

    geometry_msgs::TransformStamped bf2person = generate_tf(p);

    ros::Time obj_ts = ros::Time::now();

    while (ros::ok())
    {
      if ((ros::Time::now() - obj_ts).toSec() > OBJ_PUB_RATE)
      {
        obj_ts = ros::Time::now();
        bf2person = generate_tf(p);
      }

      try
      {
        bf2person.header.stamp = ros::Time::now();
        br.sendTransform(bf2person);
      }
      catch(tf2::TransformException &exception)
      {
        ROS_ERROR("%s", exception.what());
      }
    }
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_tf_person");

  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::Image> image_depth_sub(nh, "/camera/depth/image_raw", 1);
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub(nh, "/darknet_ros/bounding_boxes", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
  message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx(MySyncPolicy_bbx(10), image_depth_sub, bbx_sub);

  sync_bbx.registerCallback(boost::bind(&callback_bbx, _1, _2));

  ros::spin();
  
  return 0;
}