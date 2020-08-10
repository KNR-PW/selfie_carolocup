/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/

#include "ros/ros.h"
#include <intersection/intersection_server.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "intersection_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  IntersectionServer server(nh, pnh);

  ros::spin();
  return 0;
}