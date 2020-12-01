/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef VISION_VISION_MANAGER_H
#define VISION_VISION_MANAGER_H

#include <vector>
#include <vision/units/vision_unit_interface.h>
#include <sensor_msgs/Image.h>
#include <functional>
#include <vision/storage/global_storage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

class VisionManager
{
  std::vector<std::reference_wrapper<VisionUnitInterface>> units_;
public:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  GlobalStorage global_storage_;
  void init();
  void resetCallback();
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void addUnit(VisionUnitInterface &unit);
  VisionManager();
};



#endif  // VISION_VISION_MANAGER_H
