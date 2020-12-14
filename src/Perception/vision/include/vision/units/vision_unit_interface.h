/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef VISION_UNITS_VISION_UNIT_INTERFACE_H
#define VISION_UNITS_VISION_UNIT_INTERFACE_H

class VisionUnitInterface
{
public:
 virtual void init() {}
 virtual void trigger() {}
 virtual void reset() {}
};

#endif  // VISION_UNITS_VISION_UNIT_INTERFACE_H
