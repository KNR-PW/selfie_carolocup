/***Copyright ( c) 2019, KNR Selfie*
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#include <starting_procedure/starting_procedure_action.h>

StartingProcedureAction::StartingProcedureAction(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh)
  , pnh_(pnh)
  , as_(nh_, "task/starting_procedure", false)
  , min_second_press_time_(ros::Time(0))
  , debounce_duration_(ros::Duration(2))
  , distance_goal_(0.f)
  , distance_read_(0.f)
  , dr_server_CB_(boost::bind(&StartingProcedureAction::reconfigureCB, this, _1, _2))
{
  as_.registerPreemptCallback(boost::bind(&StartingProcedureAction::preemptCB, this));
  as_.registerGoalCallback(boost::bind(&StartingProcedureAction::executeCB, this));

  qr_start_search_ = nh_.serviceClient<std_srvs::Empty>("startQrSearch");
  qr_stop_search_ = nh_.serviceClient<std_srvs::Empty>("stopQrSearch");
  scan_client_ = nh_.serviceClient<std_srvs::Empty>("startGateScan");
  drive_pub_ = nh_.advertise<custom_msgs::DriveCommand>("drive_starting", 1);
  pnh_.param<float>("starting_speed", starting_speed_, 2.f);
  pnh_.param<bool>("use_scan", use_scan_, false);
  pnh_.param<bool>("use_qr", use_qr_, true);
  pnh_.param<float>("Kp", Kp_, 1.0);

  dr_server_.setCallback(dr_server_CB_);

  ROS_INFO("Starting_speed: %.3f", starting_speed_);
  ROS_INFO("use_qr: %d", use_qr_);
  ROS_INFO("use_scan: %d", use_scan_);
  as_.start();
  ROS_INFO("Starting procedure object created");
}

void StartingProcedureAction::executeCB()
{
  custom_msgs::startingGoal goal = *as_.acceptNewGoal();
  distance_goal_ = goal.distance;
  ROS_INFO("received goal %f", goal.distance);
  if (use_qr_)
  {
    qr_sub_ = nh_.subscribe("qr_gate_open", 1, &StartingProcedureAction::gateOpenCB, this);
  }
  if (use_scan_)
  {
    gate_scan_sub_ = nh_.subscribe("scan_gate_open", 1, &StartingProcedureAction::gateOpenCB, this);
  }
  distance_sub_ = nh_.subscribe("selfie_out/motion", 10, &StartingProcedureAction::distanceCB, this);
  button_sub_ = nh_.subscribe("/selfie_out/buttons", 10, &StartingProcedureAction::ButtonCB, this);
  odom_sub_ = nh_.subscribe("/odom", 10, &StartingProcedureAction::odomCallback, this);
  publishFeedback(SELFIE_READY);
  state_ = State::WAIT_BUTTON;
}

void StartingProcedureAction::ButtonCB(const custom_msgs::Buttons& msg)
{
  Buttons pressed_button;
  if (msg.is_pressed_first && !msg.is_pressed_second)
  {
    pressed_button = PARKING;
    result_.drive_mode = false;
  }
  else if (!msg.is_pressed_first && msg.is_pressed_second)
  {
    pressed_button = FREE_DRIVE;
    result_.drive_mode = true;
  }
  else
    return;

  if (state_ == State::WAIT_BUTTON)
  {
    min_second_press_time_ = ros::Time::now() + debounce_duration_;
    button_status_ = pressed_button == PARKING ? BUTTON_PARKING_DRIVE_PRESSED : BUTTON_OBSTACLE_DRIVE_PRESSED;
    if (pressed_button == PARKING)
    {
      ROS_INFO("parking button pressed for the 1st time");
    }
    else
    {
      ROS_INFO("obstacle button pressed for the 1st time");
    }
    if (use_qr_)
    {
      ROS_INFO("start qr search");
      std_srvs::Empty call = std_srvs::Empty();
      qr_start_search_.call(call);
    }
    if (use_scan_)
    {
      ROS_INFO("start scan search");
      std_srvs::Empty call = std_srvs::Empty();
      scan_client_.call(call);
    }
    state_ = State::WAIT_START;
  }
  else if (state_ == State::WAIT_START)
  {
    if (ros::Time::now() > min_second_press_time_)
    {
      if (use_qr_)
      {
        std_srvs::Empty call = std_srvs::Empty();
        qr_stop_search_.call(call);
      }
      publishFeedback(pressed_button == PARKING ? BUTTON_PARKING_DRIVE_PRESSED : BUTTON_OBSTACLE_DRIVE_PRESSED);
      distance_goal_ = distance_read_ + distance_goal_;
      starting_distance_ = distance_read_;
      if (pressed_button == PARKING)
      {
        ROS_INFO("start parking competition");
      }
      else
      {
        ROS_INFO("start obstacles competition");
      }
      init_pose_ = current_pose_;
      state_ = State::START_MOVE;
      publishFeedback(START_DRIVE);
    }
  }
}

void StartingProcedureAction::gateOpenCB(const std_msgs::Empty& msg)
{
  if (state_ == State::WAIT_START)
  {
    state_ = State::START_MOVE;
    publishFeedback(button_status_);
    distance_goal_ = distance_read_ + distance_goal_;
    ROS_INFO("gate was opened");
    starting_distance_ = distance_read_;
    init_pose_ = current_pose_;
    publishFeedback(START_DRIVE);
  }
}

void StartingProcedureAction::preemptCB()
{
  as_.setAborted();
  state_ = State::IDLE;
  ROS_INFO("STARTING PROCEDURE ABORTED");
  button_sub_.shutdown();
  distance_sub_.shutdown();
  odom_sub_.shutdown();
  if (use_qr_)
    qr_sub_.shutdown();
  if (use_scan_)
    gate_scan_sub_.shutdown();
  as_.setAborted();
}

void StartingProcedureAction::distanceCB(const custom_msgs::Motion& msg)
{
  distance_read_ = msg.distance;
  if (state_ == State::START_MOVE)
  {
    driveBoxOut(starting_speed_);
    if (distance_read_ > distance_goal_)
    {
      state_ = State::IDLE;
      ROS_INFO("starting procedure completed");
      publishFeedback(END_DRIVE);
      button_sub_.shutdown();
      distance_sub_.shutdown();
      odom_sub_.shutdown();
      if (use_qr_)
        qr_sub_.shutdown();
      if (use_scan_)
        gate_scan_sub_.shutdown();
      as_.setSucceeded(result_);
    }
  }
}

void StartingProcedureAction::odomCallback(const nav_msgs::Odometry& msg)
{
  tf::poseMsgToTF(msg.pose.pose, current_pose_);
}

void StartingProcedureAction::publishFeedback(feedback_variable program_state)
{
  feedback_.action_status = program_state;
  as_.publishFeedback(feedback_);
}

void StartingProcedureAction::driveBoxOut(float speed)
{
  custom_msgs::DriveCommand cmd;
  cmd.speed = speed;
  tf::Vector3 pos;
  pos = (init_pose_.inverse() * current_pose_).getOrigin();
  cmd.steering_angle_front = -Kp_ * pos.y();
  cmd.steering_angle_rear = 0;
  cmd.acceleration = 0;  // max acceleration
  drive_pub_.publish(cmd);
}

void StartingProcedureAction::reconfigureCB(starting_procedure::StartingProcedureConfig& config, uint32_t level)
{
  if (Kp_ != static_cast<float>(config.Kp))
  {
    Kp_ = config.Kp;
    ROS_INFO("starting procedure Kp new value: %f\n", Kp_);
  }
}
