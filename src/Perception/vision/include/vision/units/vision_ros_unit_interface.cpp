/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/
#include <vision/units/vision_ros_unit_interface.h>

template<typename T>
VisionROSUnitInterface<T>::VisionROSUnitInterface(const std::string & name): nh_(name), dr_server_(nh_)
{
}
