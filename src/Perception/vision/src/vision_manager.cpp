/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#include <vision/vision_manager.h>
#include <vision/units/vision_unit_interface.h>
#include <vision/units/homography_unit.h>
#include <vision/units/homography_unit.h>

void VisionManager::init()
{
  image_sub_ = it_.subscribe("camera_basler/image_rect", 10, &VisionManager::imageCallback, this);
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

void VisionManager::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  global_storage_.image_rect_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1)->image;
  for (VisionUnitInterface& unit : units_)
  {
    unit.trigger();
  }
}

void VisionManager::addUnit(VisionUnitInterface & unit)
{
  std::reference_wrapper<VisionUnitInterface> wrap(unit);
  units_.push_back(wrap);
}

VisionManager::VisionManager() : nh_(), it_(nh_)
{
  global_storage_ = GlobalStorage();
}




