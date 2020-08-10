/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/
#include <park/park.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "park");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  Park park(nh, pnh);
  ros::spin();

  return 0;
}
