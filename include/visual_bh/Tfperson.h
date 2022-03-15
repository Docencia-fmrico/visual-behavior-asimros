#ifndef VISUAL_BH_TFPERSON_H
#define VISUAL_BH_TFPERSON_H

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

namespace visual_bh
{

class Tfperson
{
  public:
    Tfperson(bool update);

    bool get_update() {return update_;}
    geometry_msgs::TransformStamped generate_tf();
    void callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);

  private:
    float x_;
    float y_;
    bool update_;
};

}

#endif  // VISUAL_BEHAVIOUR_ASIMROS_DATA_TF_H