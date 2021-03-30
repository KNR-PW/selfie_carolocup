/**
 *Copyright ( c ) 2019, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#include "ros/ros.h"
#include <iostream>
#include <vector>

#include <parking_spot_detector/search_server.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task/parking_spot_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  SearchServer parking_node(nh, pnh);

  ros::spin();
  return 0;
}