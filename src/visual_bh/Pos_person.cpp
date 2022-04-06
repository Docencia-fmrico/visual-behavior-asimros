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
: image_depth_sub(nh_, "/camera/depth/image_raw", 1),
bbx_sub(nh_, "/darknet_ros/bounding_boxes", 1),
sync_bbx(MySyncPolicy_bbx(10), image_depth_sub, bbx_sub),
objectFrameId_("/person"),
workingFrameId_("/base_footprint")
{
    sub_cam_ = nh_.subscribe("/camera/depth/camera_info", 1, &Pos_person::callback_caminfo, this);
    sync_bbx.registerCallback(boost::bind(&Pos_person::callback_bbx, this, _1, _2));
}

void Pos_person::callback_caminfo(const sensor_msgs::CameraInfoConstPtr& msg)
{
    cammodel_.fromCameraInfo(msg);
    xyz_ = cammodel_.projectPixelTo3dRay(pixel_);

    transform_.setOrigin(tf::Vector3(dist_, xyz_.x, xyz_.y));
    transform_.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

    transform_.stamp_ = ros::Time::now();
    transform_.frame_id_ = workingFrameId_;
    transform_.child_frame_id_ = objectFrameId_;

    try
    {
        tfBroadcaster_.sendTransform(transform_);
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
    }
    
}

void
Pos_person::callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
{
    cv_bridge::CvImagePtr img_ptr_depth;

    try
    {
        img_ptr_depth = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    for (const auto & box : boxes->bounding_boxes)
    {
        pixel_.x = (box.xmax + box.xmin) / 2;
        pixel_.y = (box.ymax + box.ymin) / 2;
        dist_ = img_ptr_depth->image.at<float>(cv::Point(pixel_.x, pixel_.y));
    }
}

}  // namespace visual_bh
