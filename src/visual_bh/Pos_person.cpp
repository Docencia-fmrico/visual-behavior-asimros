#include "visual_bh/Pos_person.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include "ros/ros.h"

namespace visual_bh
{

Pos_person::Pos_person()
: image_depth_sub(nh, "/camera/depth/image_raw", 1), bbx_sub(nh, "/darknet_ros/bounding_boxes", 1), 
sync_bbx(MySyncPolicy_bbx(10), image_depth_sub, bbx_sub)
{
    sync_bbx.registerCallback(boost::bind(&Pos_person::callback_bbx, this, _1, _2));
}

void 
Pos_person::callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
{
    cv_bridge::CvImagePtr img_ptr_depth;
    ROS_INFO("estoy en el callback");
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

        x_ = img_ptr_depth->image.at<float> (cv::Point(px, py));
        y_ = px;
    }
}

}