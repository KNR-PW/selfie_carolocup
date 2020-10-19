/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef CUSTOM_MSGS_LANE_CONTROL_ENUM_H
#define CUSTOM_MSGS_LANE_CONTROL_ENUM_H

// published on topic:
// /state/lane_control
namespace selfie {
typedef enum EnumLaneControl
{
  UNINITIALIZED = 0, // lane_controller is disabled
  PASSIVE_RIGHT,
  ON_RIGHT,
  OVERTAKE, // Changing lane from left ro right one
  ON_LEFT,
  RETURN_RIGHT // Returning to right lane from left one
} EnumLaneControl;
} // namespace selfie

#endif // CUSTOM_MSGS_LANE_CONTROL_ENUM_H
