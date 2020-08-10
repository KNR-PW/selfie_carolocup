/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#include <park/park.h>

Park::Park(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), as_(nh_, "park", false), dr_server_CB_(boost::bind(&Park::reconfigureCB, this, _1, _2))
{
  pnh_.param<bool>("state_msgs", state_msgs_, false);
  pnh_.param<float>("parking_speed", parking_speed_, 0.8);
  pnh_.param<float>("max_turn", max_turn_, 0.5);
  pnh_.param<float>("idle_time", idle_time_, 2.);
  pnh_.param<float>("iter_distance", iter_distance_, 0.2);
  pnh_.param<float>("angle_coeff", angle_coeff_, 1. / 2.);
  pnh_.param<float>("back_to_mid", back_to_mid_, 0.18);
  pnh_.param<float>("turn_delay", turn_delay_, 0.1);
  pnh_.param<float>("line_dist_end", line_dist_end_, 0.15);
  pnh_.param<float>("start_parking_speed", start_parking_speed_, 0.5);

  park_spot_middle_ = 0.;
  front_target_ = 0.;
  back_target_ = 0.;
  out_target_ = 0.;
  delay_end_ = ros::Time::now();
  dr_server_.setCallback(dr_server_CB_);
  move_state_ = first_phase;
  parking_state_ = not_parking;
  as_.registerGoalCallback(boost::bind(&Park::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&Park::preemptCB, this));
  steering_mode_set_parallel_ = nh_.serviceClient<std_srvs::Empty>("steering_parallel");
  steering_mode_set_front_axis_ = nh_.serviceClient<std_srvs::Empty>("steering_front_axis");
  as_.start();
  dist_sub_ = nh_.subscribe("/distance", 1, &Park::distanceCallback, this);
  ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/drive_park", 10);
  right_indicator_pub_ = nh_.advertise<std_msgs::Bool>("right_turn_indicator", 20);
  left_indicator_pub_ = nh_.advertise<std_msgs::Bool>("left_turn_indicator", 20);
  markings_sub_ = nh_.subscribe("/road_markings", 10, &Park::markingsCallback, this);
}

void Park::distanceCallback(const std_msgs::Float32& msg)
{
  actual_dist_ = msg.data;
  custom_msgs::parkFeedback feedback;
  switch (parking_state_)
  {
    case go_to_parking_spot:
      if (toParkingSpot())
      {
        prev_dist_ = actual_dist_;
        delay_end_ = ros::Time::now() + ros::Duration(turn_delay_);
        std_srvs::Empty empty = std_srvs::Empty();
        steering_mode_set_parallel_.call(empty);
        parking_state_ = going_in;
      }
      if (state_msgs_)
        ROS_INFO_THROTTLE(5, "go_to_parking_spot");
      blinkRight(true);
      blinkLeft(false);
      break;

    case going_in:
      if (state_msgs_)
        ROS_INFO_THROTTLE(5, "get_in");
      if (park())
        parking_state_ = parked;
      blinkRight(true);
      blinkLeft(false);
      break;

    case parked:
      if (state_msgs_)
        ROS_INFO_THROTTLE(5, "parked");
      drive(0., 0.);
      blinkLeft(false);
      blinkRight(false);
      blinkLeft(true);
      blinkRight(true);
      feedback.action_status = IN_PLACE;
      as_.publishFeedback(feedback);
      ros::Duration(idle_time_).sleep();
      parking_state_ = going_out;
      break;

    case going_out:
      blinkLeft(true);
      blinkRight(false);
      if (state_msgs_)
        ROS_INFO_THROTTLE(5, "get_out");
      if (leave())
        parking_state_ = out;
      break;

    case out:
      feedback.action_status = OUT_PLACE;
      as_.publishFeedback(feedback);
      blinkLeft(false);
      blinkRight(false);
      if (state_msgs_)
        ROS_INFO_THROTTLE(5, "out");
      drive(parking_speed_, 0);
      feedback.action_status = READY_TO_DRIVE;
      as_.publishFeedback(feedback);
      custom_msgs::parkResult result;
      result.done = true;
      as_.setSucceeded(result);
      parking_state_ = not_parking;
      break;
  }
}

void Park::goalCB()
{
  custom_msgs::parkGoal goal = *as_.acceptNewGoal();
  initParkingSpot(goal.parking_spot);
  custom_msgs::parkFeedback feedback;
  feedback.action_status = START_PARK;
  as_.publishFeedback(feedback);
  parking_state_ = go_to_parking_spot;
  std_srvs::Empty empty = std_srvs::Empty();
  steering_mode_set_front_axis_.call(empty);
}

void Park::preemptCB()
{
  ROS_INFO("parkService preempted");
  blinkLeft(false);
  blinkRight(false);
  parking_state_ = not_parking;
  move_state_ = first_phase;
  as_.setAborted();
}

void Park::initParkingSpot(const geometry_msgs::Polygon& msg)
{
  park_spot_middle_ = (msg.points[0].x + msg.points[3].x) / 2.;
  float mid_on_line(0.);
  float powered_x = 1.;
  for (float& coef : right_line_)
  {
    mid_on_line += coef * powered_x;
    powered_x *= park_spot_middle_;
  }
  park_spot_dist_ = std::abs(mid_on_line - PARK_SPOT_WIDTH / 2.);

  out_target_ = std::abs(mid_on_line) + line_dist_end_;
  back_target_ = actual_dist_ + park_spot_middle_ - iter_distance_ / 2. - back_to_mid_;
  front_target_ = back_target_ + iter_distance_;
}

void Park::drive(float speed, float steering_angle)
{
  ackermann_msgs::AckermannDriveStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.drive.speed = speed;
  msg.drive.steering_angle = steering_angle;
  ackermann_pub_.publish(msg);
}

bool Park::toParkingSpot()
{
  if (actual_dist_ > back_target_)
  {
    drive(0., -max_turn_);
    return true;
  }
  drive(start_parking_speed_, 0.);
  return false;
}

bool Park::park()
{
  park_spot_dist_ -= angle_coeff_ * std::sin(max_turn_) * std::abs(actual_dist_ - prev_dist_);
  bool in_pos = park_spot_dist_ > 0.f;

  if (move_state_ == first_phase)
  {
    if (in_pos)
    {
      if (actual_dist_ > front_target_)
      {
        move_state_ = second_phase;
        delay_end_ = ros::Time::now() + ros::Duration(turn_delay_);
        drive(0., max_turn_);
      }
      else if (ros::Time::now() > delay_end_)
      {
        drive(parking_speed_, -max_turn_);
      }
    }
    else
    {
      move_state_ = second_phase;
      return true;
    }
  }
  else
  {
    if (in_pos)
    {
      if (actual_dist_ < back_target_)
      {
        move_state_ = first_phase;
        delay_end_ = ros::Time::now() + ros::Duration(turn_delay_);
        drive(0., -max_turn_);
      }
      else if (ros::Time::now() > delay_end_)
      {
        drive(-parking_speed_, max_turn_);
      }
    }
    else
    {
      move_state_ = first_phase;
      return true;
    }
  }
  prev_dist_ = actual_dist_;
  return false;
}

bool Park::leave()
{
  static ros::Time delay_end;

  park_spot_dist_ += angle_coeff_ * std::sin(max_turn_) * std::abs(actual_dist_ - prev_dist_);
  bool in_pos = park_spot_dist_ < out_target_;
  if (move_state_ == first_phase)
  {
    if (in_pos)
    {
      if (actual_dist_ > front_target_)
      {
        move_state_ = second_phase;
        delay_end = ros::Time::now() + ros::Duration(turn_delay_);
        drive(0., -max_turn_);
      }
      else if (ros::Time::now() > delay_end)
      {
        drive(parking_speed_, max_turn_);
      }
    }
    else
    {
      move_state_ = second_phase;
      return true;
    }
  }
  else
  {
    if (in_pos)
    {
      if (actual_dist_ < back_target_)
      {
        move_state_ = first_phase;
        delay_end = ros::Time::now() + ros::Duration(turn_delay_);
        drive(0., max_turn_);
      }
      else if (ros::Time::now() > delay_end)
      {
        drive(-parking_speed_, -max_turn_);
      }
    }
    else
    {
      move_state_ = first_phase;
      return true;
    }
  }
  prev_dist_ = actual_dist_;
  return false;
}

void Park::markingsCallback(const custom_msgs::RoadMarkings& msg)
{
  if (parking_state_ == not_parking)
  {
    right_line_ = msg.right_line;
  }
}

void Park::blinkLeft(bool on)
{
  std_msgs::Bool msg;
  msg.data = on;
  left_indicator_pub_.publish(msg);
  return;
}

void Park::blinkRight(bool on)
{
  std_msgs::Bool msg;
  msg.data = on;
  right_indicator_pub_.publish(msg);
  return;
}

void Park::reconfigureCB(park::ParkConfig& config, uint32_t level)
{
  if (idle_time_ != static_cast<float>(config.idle_time))
  {
    idle_time_ = config.idle_time;
    ROS_INFO("idle_time_ new value: %f", idle_time_);
  }
  if (max_turn_ != static_cast<float>(config.max_turn))
  {
    max_turn_ = config.max_turn;
    ROS_INFO("max_turn new value: %f", max_turn_);
  }

  if (parking_speed_ != static_cast<float>(config.parking_speed))
  {
    parking_speed_ = config.parking_speed;
    ROS_INFO("parking_speed_ new value: %f", parking_speed_);
  }
  if (start_parking_speed_ != static_cast<float>(config.start_parking_speed))
  {
    start_parking_speed_ = config.start_parking_speed;
    ROS_INFO("start_parking_speed_ new value: %f", start_parking_speed_);
  }
  if (angle_coeff_ != static_cast<float>(config.angle_coeff))
  {
    angle_coeff_ = config.angle_coeff;
    ROS_INFO("angle_coeff_ new value: %f", angle_coeff_);
  }
  if (iter_distance_ != static_cast<float>(config.iter_distance))
  {
    iter_distance_ = config.iter_distance;
    ROS_INFO("iter_distance_ new value: %f", iter_distance_);
  }
  if (turn_delay_ != static_cast<float>(config.turn_delay))
  {
    turn_delay_ = config.turn_delay;
    ROS_INFO("turn_delay_ new value: %f", turn_delay_);
  }
  if (back_to_mid_ != static_cast<float>(config.back_to_mid))
  {
    back_to_mid_ = config.back_to_mid;
    ROS_INFO("back_to_mid_ new value: %f", back_to_mid_);
  }
  if (line_dist_end_ != static_cast<float>(config.line_dist_end))
  {
    line_dist_end_ = config.line_dist_end;
    ROS_INFO("line_dist_end_ new value: %f", line_dist_end_);
  }
}
