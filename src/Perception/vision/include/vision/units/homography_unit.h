/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef VISION_UNITS_HOMOGRAPHY_UNIT_H
#define VISION_UNITS_HOMOGRAPHY_UNIT_H
#include <vision/units/vision_ros_unit_interface.h>
#include <vision/storage/homography_storage.h>
#include <vision/HomographyConfig.h>

class HomographyUnit : public VisionROSUnitInterface<vision::HomographyConfig>
{
  HomographyStorage storage_;
};

#endif  // VISION_UNITS_HOMOGRAPHY_UNIT_H
