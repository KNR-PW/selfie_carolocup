/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#include <vision/vision_manager.h>
#include <vision/units/vision_unit_interface.h>

void VisionManager::init()
{
  for (VisionUnitInterface& unit : units_)
  {
    unit.init();
  }
}

void VisionManager::resetCallback()
{
  for (VisionUnitInterface& unit : units_)
  {
    unit.reset();
  }
}

void VisionManager::imageCallback(const sensor_msgs::Image& msg)
{
  for (VisionUnitInterface& unit : units_)
  {
    unit.trigger();
  }
}
