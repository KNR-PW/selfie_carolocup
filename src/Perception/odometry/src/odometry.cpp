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
  , roll_(0.)
  , pitch_(0.)
  , yaw_(0.)
  , base_yaw_(0.)
  , yaw_initialized_(false)
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
  , distance_initialized_(false)
  , listener_()
  , odom_broadcaster_()
{
  odom_quat_ = tf::createQuaternionMsgFromYaw(yaw_);
  pub_odom_ = nh_.advertise<nav_msgs::Odometry>("/odom", 50);
  sub_distance_ = nh_.subscribe("/distance", 50, &Odometry::distanceCallback, this);
  sub_imu_ = nh_.subscribe("/imu", 50, &Odometry::imuCallback, this);
  rear_axis_frame_ = nh_.param<std::string>("rear_axis_frame", "base_link");

  listener_.waitForTransform(rear_axis_frame_, IMU_FRAME, ros::Time::now(), ros::Duration(3.0));
  listener_.lookupTransform(rear_axis_frame_, IMU_FRAME, ros::Time(0), imu_transform_);
}

void Odometry::distanceCallback(const std_msgs::Float32& msg)
{
  current_time_ = ros::Time::now();
  currentDistance_ = msg.data;

  if (distance_initialized_)
  {
    deltaDistance_ = currentDistance_ - lastDistance_;
    dt_ = (current_time_ - last_distance_update_).toSec();
    if (dt_ != 0)
    {
      speed_ = deltaDistance_ / dt_;
      vx_ = speed_ * cos(yaw_);
      vy_ = speed_ * sin(yaw_);
    }

    dx_ = deltaDistance_ * cos(yaw_);
    dy_ = deltaDistance_ * sin(yaw_);

    x_ += dx_;
    y_ += dy_;
  }

  distance_initialized_ = true;
  last_distance_update_ = current_time_;
  lastDistance_ = currentDistance_;
}

void Odometry::imuCallback(const sensor_msgs::Imu& msg)
{
  tf::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);

  q = imu_transform_ * q;

  tf::Matrix3x3 m(q);
  m.getRPY(roll_, pitch_, yaw_);

  if (!yaw_initialized_)
  {
    base_yaw_ = yaw_;
    yaw_initialized_ = true;
  }

  yaw_ -= base_yaw_;
  vyaw_ = msg.angular_velocity.z;

  // quaternion created from yaw
  odom_quat_ = tf::createQuaternionMsgFromYaw(yaw_);
}

void Odometry::publishOdometryTransform()
{
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = ODOM_FRAME;
  odom_trans.child_frame_id = rear_axis_frame_;

  odom_trans.transform.translation.x = x_;
  odom_trans.transform.translation.y = y_;
  odom_trans.transform.translation.z = 0.0;
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
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat_;

  // set the velocity
  odom.child_frame_id = rear_axis_frame_;
  odom.twist.twist.linear.x = vx_;
  odom.twist.twist.linear.y = vy_;
  odom.twist.twist.angular.z = vyaw_;

  // publish the message
  pub_odom_.publish(odom);
}
