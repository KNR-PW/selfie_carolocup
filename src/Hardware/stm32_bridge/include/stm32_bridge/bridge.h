/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef STM32_BRIDGE_BRIDGE_H
#define STM32_BRIDGE_BRIDGE_H

#include "ros/ros.h"
#include "custom_msgs/Motion.h"
#include "custom_msgs/Buttons.h"
#include "custom_msgs/Indicators.h"

class Time
{
    const ros::Time begin;

public:
    uint32_t get_ms_time();
    Time();
};

class Ackermann_control
{
public:
    float steering_angle_front;
    float steering_angle_rear;
    float speed;
    float acceleration;
    float jerk;
};

class Indicator_control
{
public:
    uint8_t left;
    uint8_t right;
};

class Pub_messages
{
public:
    custom_msgs::Motion motion_msg;
    custom_msgs::Buttons buttons_msg;

    int rc_state;
};

class Sub_messages
{
public:
    Ackermann_control ackerman;
    Sub_messages();
};

#endif  // STM32_BRIDGE_BRIDGE_H
