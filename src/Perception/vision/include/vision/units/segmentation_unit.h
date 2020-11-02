
#ifndef SEGMENTATION_UNIT_H
#define SEGMENTATION_UNIT_H
#include "vision_ros_unit_interface.h"
#include <vision/storage/segmentation_storage.h>
#include <vision/SegmentationConfig.h>

class SegmentationUnit: public VisionROSUnitInterface<vision::SegmentationConfig>
{
  SegmentationStorage storage_;

};


#endif

