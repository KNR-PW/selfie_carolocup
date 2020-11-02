/**
 * Copyright ( c ) 2019, KNR Selfie
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#include <odometry/odometry.h>
#include <string>

Odometry::Odometry(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh)
  , pnh_(pnh)
  , speed_(0.)
  , yaw_(0.)
  , base_yaw_(0.)
  , initialized_(false)
  , x_(0.)
  , y_(0.)
  , vx_(0.)
  , vy_(0.)
  , vyaw_(0.)
  , currentDistance_(0.)
  , lastDistance_(0.)
  , deltaDistance_(0.)
  , dt_(0.)
  , dx_(0.)
  , dy_(0.)
  , odom_broadcaster_()
{
  odom_quat_ = tf::createQuaternionMsgFromYaw(yaw_);
  pub_odom_ = nh_.advertise<nav_msgs::Odometry>("/odom", 50);
  sub_motion_ = nh_.subscribe("/selfie_out/motion", 50, &Odometry::motionCallback, this);
  rear_axis_frame_ = nh_.param<std::string>("rear_axis_frame", "base_link");
  reset_odom_ = nh_.advertiseService("/reset/odom", &Odometry::resetOdom, this);
}

void Odometry::motionCallback(const custom_msgs::Motion& msg)
{
  current_time_ = ros::Time::now();
  currentDistance_ = msg.distance;
  yaw_ = angles::normalize_angle(msg.yaw - base_yaw_);
  vyaw_ = msg.speed_yaw;
  odom_quat_ = tf::createQuaternionMsgFromYaw(yaw_);

  if (!initialized_)
  {
    base_yaw_ = yaw_;
    initialized_ = true;
  }
  else
  {
    deltaDistance_ = currentDistance_ - lastDistance_;
    dt_ = (current_time_ - last_distance_update_).toSec();
    if (dt_ != 0.)
    {
      speed_ = deltaDistance_ / dt_;
      vx_ = speed_ * cos(yaw_);
      vy_ = speed_ * sin(yaw_);
    }

    dx_ = deltaDistance_ * cos(yaw_);
    dy_ = deltaDistance_ * sin(yaw_);

    x_ += dx_;
    y_ += dy_;

    publishOdometryTransform();
    publishOdometryMessage();
  }
  last_distance_update_ = current_time_;
  lastDistance_ = currentDistance_;
}

void Odometry::publishOdometryTransform()
{
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = ODOM_FRAME;
  odom_trans.child_frame_id = rear_axis_frame_;

  odom_trans.transform.translation.x = x_;
  odom_trans.transform.translation.y = y_;
  odom_trans.transform.translation.z = 0.;
  odom_trans.transform.rotation = odom_quat_;

  // send the transform
  odom_broadcaster_.sendTransform(odom_trans);
}

void Odometry::publishOdometryMessage()
{
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = ODOM_FRAME;

  // set the position
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.;
  odom.pose.pose.orientation = odom_quat_;

  // set the velocity
  odom.child_frame_id = rear_axis_frame_;
  odom.twist.twist.linear.x = vx_;
  odom.twist.twist.linear.y = vy_;
  odom.twist.twist.angular.z = vyaw_;

  // publish the message
  pub_odom_.publish(odom);
}

bool Odometry::resetOdom(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  base_yaw_ = angles::normalize_angle(yaw_ + base_yaw_);
  x_ = 0.;
  y_ = 0.;
  ROS_INFO("odometry is reset");
  return true;
}
