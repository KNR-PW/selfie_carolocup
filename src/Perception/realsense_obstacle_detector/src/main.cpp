#include <realsense_obstacle_detector/RealSensePcl.hpp>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "realsense_obstacle_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  RealSensePcl sample(nh, pnh);

  ros::spin();
  return 0;
}