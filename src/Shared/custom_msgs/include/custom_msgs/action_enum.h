/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef CUSTOM_MSGS_ACTION_ENUM_H
#define CUSTOM_MSGS_ACTION_ENUM_H

namespace selfie
{
typedef enum EnumAction
{
  ACTION_NONE = 0,
  STARTING_PROCEDURE,
  FREE_DRIVE,
  PARKING_SPOT_SEARCH,
  PARK,
  INTERSECTION_STOP,
} EnumAction;
}  // namespace selfie

#endif  // CUSTOM_MSGS_ACTION_ENUM_H
