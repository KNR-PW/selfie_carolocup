/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef CUSTOM_MSGS_RC_ENUM_H
#define CUSTOM_MSGS_RC_ENUM_H

// published on topic:
// /state/rc
namespace selfie {
typedef enum EnumRC
{
  RC_UNINITIALIZED = 0,
  RC_MANUAL,
  RC_HALF_AUTONOMOUS,
  RC_AUTONOMOUS
} EnumRC;
} // namespace selfie

#endif // CUSTOM_MSGS_RC_ENUM_H
