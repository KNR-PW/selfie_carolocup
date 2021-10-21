/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float32.h>

#include <custom_msgs/RoadLines.h>
#include <custom_msgs/enums.h>
#include <vision/VisionConfig.h>
#include <vision/vision.h>

float _some_param;
float _other_param;
bool _visualization;

void reconfigureCB(vision::VisionConfig& config, uint32_t level)
{
  if (_some_param != config.some_param)
  {
    _some_param = config.some_param;
    ROS_INFO("some_param new value: %f", _some_param);
  }
  if (_other_param != config.other_param)
  {
    _other_param = config.other_param;
    ROS_INFO("other_param new value: %f", _other_param);
  }
}

void someCallback(const std_msgs::Float32& msg)
{
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.getParam("some_param", _some_param);
  pnh.getParam("other_param", _other_param);
  pnh.getParam("visualization", _visualization);

  dynamic_reconfigure::Server<vision::VisionConfig> server;
  dynamic_reconfigure::Server<vision::VisionConfig>::CallbackType f;

  f = boost::bind(&reconfigureCB, _1, _2);
  server.setCallback(f);

  ros::Publisher some_pub = nh.advertise<std_msgs::Float32>("some_output", 100);
  ros::Subscriber some_sub = nh.subscribe("/some_input", 10, &someCallback);

  feedback_variable program_state = AUTONOMOUS_DRIVE;


  while (ros::ok())
  {
    ros::spin();
  }

  return 0;
}
