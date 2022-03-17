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

#ifndef BALL_POSITIONING_IMAGE_FILTER_H
#define BALL_POSITIONING_IMAGE_FILTER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32.h>

namespace ball_positioning
{

enum {IDX_h, IDX_H, IDX_s, IDX_S, IDX_v, IDX_V, NUM_HSV};

typedef struct
{
  float hsv[NUM_HSV];
  ros::Publisher hsv_pubs[NUM_HSV];
}
HSVInfo;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  int hupper_, hlower_;
  int supper_, slower_;
  int vupper_, vlower_;
  int channel_;

  static const int MAX_CHANNELS = 3;
  HSVInfo hsvValues_[MAX_CHANNELS];

public:
  ImageConverter();
  ~ImageConverter();
  void initChannel(HSVInfo * hsvinfo, int i);
  void imageCb(const sensor_msgs::Image::ConstPtr& msg);
  void publishHSV();
};

}  // namespace ball_positioning

#endif  // BALL_POSITIONING_IMAGE_FILTER_H
