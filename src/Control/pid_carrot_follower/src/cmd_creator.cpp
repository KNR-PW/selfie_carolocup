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

std_msgs::Float64 _speed;
std_msgs::Float64 _steering_angle;
std_msgs::Float64 _acceleration;


bool _two_axis_steering_mode = false;

void steeringCallback(const std_msgs::Float64& msg)
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

void laneControllCallback(const std_msgs::Int8& msg)
{
  std_srvs::Empty empty_msg;
  switch (msg.data)
  {
    case selfie::EnumLaneControl::OVERTAKE:
      _two_axis_steering_mode = true;
    case selfie::EnumLaneControl::RETURN_RIGHT:
      _two_axis_steering_mode = true;
      break;
    default:
      _two_axis_steering_mode = false;
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

  int publish_rate = 50;
  n.getParam("publish_rate", publish_rate);

  ros::Rate loop_rate(publish_rate);
  custom_msgs::DriveCommand drive_msg;

  _speed.data = 0;
  _steering_angle.data = 0;
  _acceleration.data = 0;

  float front_right_offset = 0.;
  float rear_right_offset = 0.;
  float front_left_offset = 0.;
  float rear_left_offset = 0.;
  n.getParam("front_right_offset", front_right_offset);
  n.getParam("rear_right_offset", rear_right_offset);
  n.getParam("front_left_offset", front_left_offset);
  n.getParam("rear_left_offset", rear_left_offset);

  while (n.ok())
  {
    // check for incoming messages
    ros::spinOnce();

    drive_msg.speed = _speed.data;
    if(_two_axis_steering_mode)
    {
      if(_steering_angle.data>0)
      {
	      drive_msg.steering_angle_front = _steering_angle.data + front_left_offset;
	      drive_msg.steering_angle_rear = _steering_angle.data + rear_left_offset;
      }
      else
      {
	      drive_msg.steering_angle_front = _steering_angle.data + front_right_offset;
	      drive_msg.steering_angle_rear = _steering_angle.data + rear_left_offset;
      }
        
    }
    else
    {
      drive_msg.steering_angle_front = _steering_angle.data;
      drive_msg.steering_angle_rear = 0;
    }

    drive_msg.acceleration = _acceleration.data;
    drive_pub.publish(drive_msg);

    loop_rate.sleep();
  }
}
