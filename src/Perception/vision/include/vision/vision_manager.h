/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef VISION_VISION_MANAGER_H
#define VISION_VISION_MANAGER_H

#include <vector>
#include <vision/units/vision_unit_interface.h>
#include <sensor_msgs/Image.h>

class VisionManager
{
  std::vector<VisionUnitInterface> units_;
  void init();
  void resetCallback();
  void imageCallback(const sensor_msgs::Image& msg);
};

#endif  // VISION_VISION_MANAGER_H
