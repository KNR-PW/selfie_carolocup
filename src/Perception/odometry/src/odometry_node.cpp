/** 
* Copyright ( c ) 2019, KNR Selfie 
* This code is licensed under BSD license (see LICENSE for details) 
**/

#include <ros/ros.h>
#include <odometry/odometry.h>
#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Rate rate(pnh.param<int>("spin_rate", 10));
  Odometry odom(nh, pnh);
  while (ros::ok())
  {
    ros::spinOnce();
    odom.publishOdometryTransform();
    odom.publishOdometryMessage();
    rate.sleep();
  }
}
