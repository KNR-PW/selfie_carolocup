/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#include <iostream>
#include <vision/units/vision_unit_interface.h>
#include <vision/units/vision_ros_unit_interface.h>
#include <vision/units/homography_unit.h>
#include <vision/storage/homography_storage.h>
#include <vision/units/lane_detector_unit.h>
#include <vision/units/segmentation_unit.h>
#include <vision/units/starting_line_unit.h>
#include <vision/units/intersection_stop_unit.h>
#include <vision/units/preprocessing_unit.h>
#include <vision/units/debugger_unit.h>
#include <std_msgs/String.h>

#include <vision/vision_manager.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vision");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  VisionManager visionManager;
  HomographyStorage hs(visionManager.global_storage_);
  HomographyUnit homo(hs);
  
  DebuggerStorage ds(visionManager.global_storage_);
  DebuggerUnit dbg(ds);

  visionManager.addUnit(homo);
  visionManager.addUnit(dbg);
  visionManager.init();

  ros::spin();

  return 0;
}
