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

#ifndef BALL_POSITIONING_CLOUD_FILTER_H
#define BALL_POSITIONING_CLOUD_FILTER_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/algorithm/string.hpp>

namespace ball_positioning
{

enum {IDX_h, IDX_H, IDX_s, IDX_S, IDX_v, IDX_V, NUM_HSV};

typedef struct
{
  float hsv[NUM_HSV];
  ros::Subscriber hsv_subs[NUM_HSV];
  ros::Publisher cloud_pub_;
}
HSVInfo;

class RGBDFilter
{
public:
  RGBDFilter();
  void cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);
  void hsvCB(const ros::MessageEvent<std_msgs::Float32 const>& event);

private:
  bool isValid(int channel, const pcl::PointXYZHSV& hsv);
  void initHSV();

  std::string cloudsource_;
  static const int MAX_CHANNELS = 1;
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  HSVInfo hsvFilters_[MAX_CHANNELS];
};

} // namespace ball_positioning

#endif // BALL_POSITIONING_CLOUD_FILTER_H
