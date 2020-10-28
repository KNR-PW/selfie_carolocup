
#ifndef VISION_ROS_UNIT_INTERFACE_H
#define VISION_ROS_UNIT_INTERFACE_H

#include "vision_unit_interface.h"
#include <dynamic_reconfigure/server.h>
#include <vision/VisionConfig.h>
#include <string>

class VisionROSUnitInterface: public  VisionUnitInterface
{

  ros::NodeHandle nh_;
  std::string name_;
  dynamic_reconfigure::Server<vision::VisionConfig> dr_server_;
  dynamic_reconfigure::Server<vision::VisionConfig>::CallbackType dr_server_CB_;

  void virtual reconfigureCallback(){};
};




#endif

