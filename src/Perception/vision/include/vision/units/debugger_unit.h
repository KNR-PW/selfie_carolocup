/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef VISION_UNITS_DEBUGGER_UNIT_H
#define VISION_UNITS_DEBUGGER_UNIT_H
#include <vision/units/vision_ros_unit_interface.h>
#include <vision/storage/debugger_storage.h>

class DebuggerUnit : public VisionUnitInterface
{
  DebuggerStorage storage_;
};

#endif  // VISION_UNITS_DEBUGGER_UNIT_H
