/**
 * Copyright ( c ) 2019, KNR Selfie
 * This code is licensed under BSD license (see LICENSE for details)
 **/
#include <obstacle_detector/obstacles_generator.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ObstaclesGenerator obstaclesGenerator(nh, pnh);
  if (obstaclesGenerator.init())
  {
    ROS_INFO("obstacle_detector initialized");
  }
  else
  {
    ROS_INFO("obstacle_detector doesn't work");
    return 1;
  }

  while (ros::ok())
  {
    ros::spin();
  }

  return 0;
}
