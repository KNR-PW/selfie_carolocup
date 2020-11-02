/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef VISION_UNITS_LANE_DETECTOR_UNIT_H
#define VISION_UNITS_LANE_DETECTOR_UNIT_H
#include <vision/units/vision_ros_unit_interface.h>
#include <vision/storage/lane_detector_storage.h>
#include <vision/LaneDetectorConfig.h>

class LaneDetectorUnit : public VisionROSUnitInterface<vision::LaneDetectorConfig>
{
  LaneDetectorStorage storage_;
};

#endif  // VISION_UNITS_LANE_DETECTOR_UNIT_H
