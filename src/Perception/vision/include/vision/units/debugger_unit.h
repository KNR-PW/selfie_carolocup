
#ifndef DEBUGGER_UNIT_H
#define DEBUGGER_UNIT_H
#include "vision_ros_unit_interface.h"
#include <vision/storage/debugger_storage.h>

class DebuggerUnit: public VisionUnitInterface
{
  DebuggerStorage storage_;

};


#endif

