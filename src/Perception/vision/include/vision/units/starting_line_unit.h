
#ifndef STARTING_LINE_UNIT_H
#define STARTING_LINE_UNIT_H
#include "vision_ros_unit_interface.h"
#include <vision/storage/starting_line_storage.h>
#include <vision/StartingLineConfig.h>

class StartingLineUnit: public VisionROSUnitInterface<vision::StartingLineConfig>
{
  StartingLineStorage storage_;

};


#endif

