/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef VISION_UNITS_STARTING_LINE_UNIT_H
#define VISION_UNITS_STARTING_LINE_UNIT_H
#include <vision/units/vision_ros_unit_interface.h>
#include <vision/storage/starting_line_storage.h>
#include <vision/StartingLineConfig.h>

class StartingLineUnit : public VisionROSUnitInterface<vision::StartingLineConfig>
{
  StartingLineStorage storage_;
};

#endif  //  VISION_UNITS_STARTING_LINE_UNIT_H
