/** 
* Copyright ( c ) 2019, KNR Selfie 
* This code is licensed under BSD license (see LICENSE for details) 
**/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include "custom_msgs/Motion.h"
#include <string>

#ifndef ODOMETRY_ODOMETRY_H
#define ODOMETRY_ODOMETRY_H

class Odometry
{
  double speed_;

  double yaw_;

  double base_yaw_;
  bool initialized_;

  double x_;
  double y_;

  double vx_;
  double vy_;
  double vyaw_;

  double currentDistance_;
  double lastDistance_;
  double deltaDistance_;
  double dt_;
  double dx_;
  double dy_;

  ros::Time current_time_, last_distance_update_;

  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_odom_;
  ros::Subscriber sub_motion_;
  geometry_msgs::Quaternion odom_quat_;

  tf::TransformBroadcaster odom_broadcaster_;
  std::string rear_axis_frame_;
  const std::string ODOM_FRAME = "odom";

  void motionCallback(const custom_msgs::Motion &msg);

public:
  void publishOdometryTransform();
  void publishOdometryMessage();
  Odometry(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
};
#endif  // ODOMETRY_ODOMETRY_H

