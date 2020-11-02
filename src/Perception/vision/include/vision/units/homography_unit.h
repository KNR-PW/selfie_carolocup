
#ifndef HOMOGRAPHY_UNIT_H
#define HOMOGRAPHY_UNIT_H
#include "vision_ros_unit_interface.h"
#include <vision/storage/homography_storage.h>
#include <vision/HomographyConfig.h>

class HomographyUnit: public VisionROSUnitInterface<vision::HomographyConfig>
{
  HomographyStorage storage_;

};


#endif

