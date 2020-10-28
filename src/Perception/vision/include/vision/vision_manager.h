#ifndef VISION_MANAGER_H
#define VISION_MANAGER_H

#include <vector>
#include <vision/units/vision_unit_interface.h>
#include <sensor_msgs/Image.h>

class VisionManager
{
  std::vector<VisionUnitInterface> units_;
  void init();
  void resetCallback();
  void imageCallback(const sensor_msgs::Image &msg);
};

#endif
