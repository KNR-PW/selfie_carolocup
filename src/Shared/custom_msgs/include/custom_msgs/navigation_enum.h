/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef CUSTOM_MSGS_NAVIGATION_ENUM_H
#define CUSTOM_MSGS_NAVIGATION_ENUM_H

// published on topic:
// /state/navigation
namespace selfie
{
typedef enum EnumNavigation
{
  UNINITIALIZED = 0,
  FOLLOW_RIGHT_UNDISTRACTED,
  OBSTACLE_ON_RIGHT,
  OBSTACLE_ON_LEFT,
  OBSTACLE_ON_BOTH
} EnumNavigation;
}  // namespace selfie
#endif  // CUSTOM_MSGS_NAVIGATION_ENUM_H
