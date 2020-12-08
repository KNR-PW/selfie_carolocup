/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#include <ros/ros.h>
#include <pid_carrot_follower/pid_tuner.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pid_tuner");
  PidTuner pidTuner;
  while(1)
  { ros::spinOnce();}
}