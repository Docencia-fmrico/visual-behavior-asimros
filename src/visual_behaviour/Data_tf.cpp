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

namespace visual_behaviour
{

Data_tf::Data_tf(bool update)
{
    update_ = update;
}

geometry_msgs::TransformStamped Data_tf::generate_tf()
{
    tf2::Stamped<tf2::Transform> object;
    object.frame_id_ = "base_footprint";
    object.stamp_ = ros::Time::now();

    std::cerr << "x(dits) " << x_ << " y(x) " << y_;

    object.setOrigin(tf2::Vector3(x_, y_, 0.0));

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    object.setRotation(q);

    geometry_msgs::TransformStamped object_msg = tf2::toMsg(object);
    object_msg.child_frame_id = "bf2person";

    update_ = false;

    return object_msg;
}

void Data_tf::callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
{
    cv_bridge::CvImagePtr img_ptr_depth;

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
        update_ = true;

        std::cerr << "estoy en el callback" << std::endl;
    }
}
}

