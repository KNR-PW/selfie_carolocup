
#ifndef LANE_DETECTOR_UNIT_H
#define LANE_DETECTOR_UNIT_H
#include "vision_ros_unit_interface.h"
#include <vision/storage/lane_detector_storage.h>

class LaneDetectorUnit: public VisionROSUnitInterface
{
  LaneDetectorStorage storage_;

};


#endif

