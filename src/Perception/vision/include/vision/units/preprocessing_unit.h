/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef VISION_UNITS_PREPROCESSING_UNIT_H
#define VISION_UNITS_PREPROCESSING_UNIT_H
#include <vision/units/vision_unit_interface.h>
#include <vision/storage/preprocessing_storage.h>

class PreprocessingUnit : public VisionUnitInterface
{
  PreprocessingStorage storage_;
};

#endif  // VISION_UNITS_PREPROCESSING_UNIT_H
