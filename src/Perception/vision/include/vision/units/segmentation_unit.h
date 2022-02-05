/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef VISION_UNITS_SEGMENTATION_UNIT_H
#define VISION_UNITS_SEGMENTATION_UNIT_H
#include <vision/units/vision_ros_unit_interface.h>
#include <vision/storage/segmentation_storage.h>
#include <vision/SegmentationConfig.h>

class SegmentationUnit : public VisionROSUnitInterface<vision::SegmentationConfig>
{
  SegmentationStorage storage_;
};

#endif  // VISION_UNITS_SEGMENTATION_UNIT_H
