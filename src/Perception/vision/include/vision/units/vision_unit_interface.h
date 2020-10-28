
#ifndef VISION_UNIT_INTERFACE_H
#define VISION_UNIT_INTERFACE_H

class VisionUnitInterface
{
public:
  void virtual init(){};
  void virtual trigger(){};
  void virtual reset(){};
};

#endif

