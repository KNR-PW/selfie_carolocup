/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/

#include <ros/ros.h>
#include <lane_pilot/lane_pilot.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lane_pilot");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  Road_obstacle_detector road_obst_detector(nh, pnh);

  ros::spin();
}
