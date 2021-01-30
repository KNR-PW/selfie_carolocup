/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>
#include <custom_msgs/DriveCommand.h>
#include <custom_msgs/lane_control_enum.h>
#include <custom_msgs/enums.h>

std_msgs::Float64 _speed;
std_msgs::Float64 _steering_angle;
std_msgs::Float64 _acceleration;

ros::ServiceClient set_ackerman_pid_settings_;
ros::ServiceClient set_default_pid_settings_;

steering_mode _steeringMode = FRONT_AXIS;

void steeringCallback(const std_msgs::Float64 &msg)
{
  _steering_angle = msg;
}

void speedCallback(const std_msgs::Float64& msg)
{
  _speed = msg;
}

void accelerationCallback(const std_msgs::Float64& msg)
{
  _acceleration = msg;
}

void laneControllCallback(const std_msgs::Int8 &msg)
{
  std_srvs::Empty empty_msg;
  switch(msg.data)
  {
    case selfie::EnumLaneControl::OVERTAKE:
    case selfie::EnumLaneControl::RETURN_RIGHT:
      _steeringMode = ACKERMANN;
      set_ackerman_pid_settings_.call(empty_msg);
    break;
    default:
      _steeringMode = FRONT_AXIS;
      set_default_pid_settings_.call(empty_msg);
    break;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_creator");

  ros::NodeHandle n("~");

  ros::Publisher drive_pub = n.advertise<custom_msgs::DriveCommand>("/drive", 50);
  ros::Subscriber sub_steering_angle = n.subscribe("/steering_angle", 50, steeringCallback);
  ros::Subscriber sub_speed = n.subscribe("/speed", 50, speedCallback);
  ros::Subscriber sub_acc = n.subscribe("/acceleration", 50, accelerationCallback);
  ros::Subscriber sub_lane_control = n.subscribe("/state/lane_control", 50, laneControllCallback);
  ros::ServiceClient set_ackerman_pid_settings_ = n.serviceClient<std_srvs::Empty>("/pidTuner/setAckerman");
  ros::ServiceClient set_default_pid_settings_ = n.serviceClient<std_srvs::Empty>("/pidTuner/setDefault");

  int publish_rate = 50;
  n.getParam("publish_rate", publish_rate);

  ros::Rate loop_rate(publish_rate);
  custom_msgs::DriveCommand drive_msg;

  _speed.data = 0;
  _steering_angle.data = 0;
  _acceleration.data = 0;
  
  while (n.ok())
  {
    // check for incoming messages
    ros::spinOnce();

    drive_msg.speed = _speed.data;
    drive_msg.steering_angle_front = _steering_angle.data;
    if (_steeringMode == ACKERMANN)
    {
      drive_msg.steering_angle_rear = -_steering_angle.data;
    }
    else 
    {
      drive_msg.steering_angle_rear = 0;
    }

    drive_msg.acceleration = _acceleration.data;
    drive_pub.publish(drive_msg);

    loop_rate.sleep();
  }
}
