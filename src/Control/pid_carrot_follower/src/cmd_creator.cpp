/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <custom_msgs/DriveCommand.h>

std_msgs::Float64 _speed;
std_msgs::Float64 _steering_angle;
std_msgs::Float64 _acceleration;

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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_creator");

  ros::NodeHandle n("~");

  ros::Publisher drive_pub = n.advertise<custom_msgs::DriveCommand>("/drive", 50);
  ros::Subscriber sub_steering_angle = n.subscribe("/steering_angle", 50, steeringCallback);
  ros::Subscriber sub_speed = n.subscribe("/speed", 50, speedCallback);
  ros::Subscriber sub_acc = n.subscribe("/acceleration", 50, accelerationCallback);

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
    drive_msg.steering_angle_rear = 0;  // TODO(someone): steering modes
    drive_msg.acceleration = _acceleration.data;
    drive_pub.publish(drive_msg);

    loop_rate.sleep();
  }
}
