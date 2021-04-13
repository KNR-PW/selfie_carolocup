/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#include <stm32_bridge/bridge.h>

Time::Time() : begin(ros::Time::now())
{
}

uint32_t Time::get_ms_time()
{
  ros::Time now = ros::Time::now();
  return (now.sec - begin.sec) * 1000 + (now.nsec / 1000000);
}

Sub_messages::Sub_messages()
{
  ackerman.steering_angle_rear = 0;
  ackerman.steering_angle_front = 0;
  ackerman.speed = 0;
  ackerman.acceleration = 0;
  ackerman.jerk = 0;
}
