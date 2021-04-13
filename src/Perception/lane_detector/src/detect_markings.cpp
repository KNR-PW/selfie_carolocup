/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#include <lane_detector/lane_detector.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect_markings");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  LaneDetector laneDetector(nh, pnh);
  ros::ServiceServer service = nh.advertiseService("resetVision", &LaneDetector::resetVisionCallback, &laneDetector);
  if (laneDetector.init())
  {
    ROS_INFO("detect_markings initialized");
  }
  else
  {
    ROS_INFO("detect_markings doesn't work");
    return 1;
  }

  while (ros::ok())
  {
    ros::spin();
  }

  return 0;
}
