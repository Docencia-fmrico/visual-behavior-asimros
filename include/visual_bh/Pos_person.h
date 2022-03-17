#ifndef VISUAL_BH_POS_PERSON_H
#define VISUAL_BH_POS_PERSON_H

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include "ros/ros.h"

namespace visual_bh
{

class Pos_person
{
  public:
    Pos_person();

    double x() {return x_;}
    double y() {return y_;}
    void callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);

  private:
    ros::NodeHandle nh;

    double x_;
    double y_;

    message_filters::Subscriber<sensor_msgs::Image> image_depth_sub;
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
    message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx;
};

}  // namespace visual_bh

#endif  // VISUAL_BH_POS_PERSON_H