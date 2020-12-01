/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef VISION_UNITS_VISION_ROS_UNIT_INTERFACE_H
#define VISION_UNITS_VISION_ROS_UNIT_INTERFACE_H

#include <vision/units/vision_unit_interface.h>
#include <dynamic_reconfigure/server.h>
#include <string>

template <typename Config>
class VisionROSUnitInterface : public VisionUnitInterface
{
protected:
  ros::NodeHandle nh_, pnh_;
  std::string name_;
  typename dynamic_reconfigure::Server<Config> dr_server_;
  typename dynamic_reconfigure::Server<Config>::CallbackType dr_server_CB_;

  void virtual reconfigureCallback() {}
public:
  VisionROSUnitInterface();
};
#include <vision/units/vision_ros_unit_interface.cpp>

#endif  // VISION_UNITS_VISION_ROS_UNIT_INTERFACE_H
