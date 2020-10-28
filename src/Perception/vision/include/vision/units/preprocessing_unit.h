
#ifndef PREPROCESSING_UNIT_H
#define PREPROCESSING_UNIT_H
#include "vision_unit_interface.h"
#include <vision/storage/preprocessing_storage.h>

class PreprocessingUnit: public VisionUnitInterface
{
  PreprocessingStorage storage_;

};


#endif

