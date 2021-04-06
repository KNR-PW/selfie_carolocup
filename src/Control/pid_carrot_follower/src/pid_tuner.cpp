/**
 * Copyright (c) 2021 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <pid_carrot_follower/pid_tuner.h>

#include <pid/PidConfig.h>
#include <dynamic_reconfigure/Config.h>

PidTuner::PidTuner() : pnh_("~"), dr_server_CB_(boost::bind(&PidTuner::reconfigureCB, this, _1, _2))
{
  dr_server_.setCallback(dr_server_CB_);

  sub_motion_ = nh_.subscribe("/selfie_out/motion", 50, &PidTuner::speedCallback, this);
  set_lane_change_pid_settings_srv_ = nh_.advertiseService("/pidTuner/setLaneChangePidSettings",
                                                           &PidTuner::setLaneChangePidSettingsCb, this);
  set_default_pid_settings_srv_ = nh_.advertiseService("/pidTuner/setDefaultPidSettings",
                                                       &PidTuner::setDefaultPidSettingsCb, this);
  use_lane_change_pid_settings_ = false;

  pnh_.getParam("pid_tuner_disabled", pid_tuner_disabled_);

  pnh_.getParam("L_Kp", L_Kp_);
  pnh_.getParam("L_Ki", L_Ki_);
  pnh_.getParam("L_Kd", L_Kd_);

  pnh_.getParam("M_Kp", M_Kp_);
  pnh_.getParam("M_Ki", M_Ki_);
  pnh_.getParam("M_Kd", M_Kd_);

  pnh_.getParam("H_Kp", H_Kp_);
  pnh_.getParam("H_Ki", H_Ki_);
  pnh_.getParam("H_Kd", H_Kd_);

  pnh_.getParam("LaneChange_Kp", LaneChange_Kp_);
  pnh_.getParam("LaneChange_Ki", LaneChange_Kp_);
  pnh_.getParam("LaneChange_Kd", LaneChange_Kp_);

  pnh_.getParam("H_speed", H_speed_);
  pnh_.getParam("M_speed", M_speed_);
  pnh_.getParam("speed_change_treshold", speed_change_treshold_);
}

void PidTuner::speedCallback(const custom_msgs::Motion& msg)
{
  if (pid_tuner_disabled_ || use_lane_change_pid_settings_)
  {
    return;
  }

  if ((abs(msg.speed_linear - act_speed_) < speed_change_treshold_))
  {
    return;
  }

  act_speed_ = msg.speed_linear;
  if (act_speed_ < M_speed_)
  {
    setKp(L_Kp_);
    setKd(L_Kd_);
    setKi(L_Ki_);
  }
  else if (act_speed_ < H_speed_)
  {
    setKp(M_Kp_);
    setKd(M_Kd_);
    setKi(M_Ki_);
  }
  else
  {
    setKp(H_Kp_);
    setKd(H_Kd_);
    setKi(H_Ki_);
  }
}

bool PidTuner::setDefaultPidSettingsCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  use_lane_change_pid_settings_ = false;
  act_speed_ = 0.0;  // to be sure that speed callback will act immediately
  ROS_INFO("Use default PID settings");
  return true;
}

bool PidTuner::setLaneChangePidSettingsCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  use_lane_change_pid_settings_ = true;
  ROS_INFO("Use LaneChange PID settings");
  setKp(LaneChange_Kp_);
  setKd(LaneChange_Kd_);
  setKi(LaneChange_Ki_);
  return true;
}

void PidTuner::setKd(float Kd)
{
  float scale = 1.0;
  while ((Kd > 1 || Kd <= 0.1) && scale > 0.1)
  {
    if (Kd > 1)
    {
      Kd = Kd / 10;
      scale = scale * 10;
    }
    else if (Kd <= 0.1)
    {
      Kd = Kd * 10;
      scale = scale / 10;
    }
  }

  conf_.doubles.clear();
  double_param_.name = "Kd";
  double_param_.value = Kd;
  conf_.doubles.push_back(double_param_);

  double_param_.name = "Kd_scale";
  double_param_.value = scale;
  conf_.doubles.push_back(double_param_);

  srv_req_.config = conf_;

  ros::service::call("/pid_controller/set_parameters", srv_req_, srv_resp_);

  pnh_.setParam("/pid_controller/Kd", Kd);
  pnh_.setParam("/pid_controller/Kd_scale", scale);
}

void PidTuner::setKp(float Kp)
{
  float scale = 1.0;
  while ((Kp > 1 || Kp <= 0.1) && scale > 0.1)
  {
    if (Kp > 1)
    {
      Kp = Kp / 10;
      scale = scale * 10;
    }
    else if (Kp <= 0.1)
    {
      Kp = Kp * 10;
      scale = scale / 10;
    }
  }

  conf_.doubles.clear();
  double_param_.name = "Kp";
  double_param_.value = Kp;
  conf_.doubles.push_back(double_param_);

  double_param_.name = "Kp_scale";
  double_param_.value = scale;
  conf_.doubles.push_back(double_param_);

  srv_req_.config = conf_;

  ros::service::call("/pid_controller/set_parameters", srv_req_, srv_resp_);

  pnh_.setParam("/pid_controller/Kp", Kp);
  pnh_.setParam("/pid_controller/Kp_scale", scale);
}

void PidTuner::setKi(float Ki)
{
  float scale = 1.0;
  while ((Ki > 1 || Ki <= 0.1) && scale > 0.1)
  {
    if (Ki > 1)
    {
      Ki = Ki / 10;
      scale = scale * 10;
    }
    else if (Ki <= 0.1)
    {
      Ki = Ki * 10;
      scale = scale / 10;
    }
  }

  conf_.doubles.clear();
  double_param_.name = "Ki";
  double_param_.value = Ki;
  conf_.doubles.push_back(double_param_);

  double_param_.name = "Ki_scale";
  double_param_.value = scale;
  conf_.doubles.push_back(double_param_);

  srv_req_.config = conf_;

  ros::service::call("/pid_controller/set_parameters", srv_req_, srv_resp_);

  pnh_.setParam("/pid_controller/Ki", Ki);
  pnh_.setParam("/pid_controller/Ki_scale", scale);
}

void PidTuner::reconfigureCB(pid_carrot_follower::PIDTunerConfig& config, uint32_t level)
{
  if (pid_tuner_disabled_ != static_cast<bool>(config.pid_tuner_disabled))
  {
    pid_tuner_disabled_ = config.pid_tuner_disabled;
    if (pid_tuner_disabled_)
      ROS_INFO("PID tuner disabled");
    else
      ROS_INFO("PID tuner enabled");
  }
  if (H_Kp_ != static_cast<float>(config.H_Kp))
  {
    H_Kp_ = config.H_Kp;
    ROS_INFO("H_Kp new value %f", H_Kp_);
  }
  if (H_Ki_ != static_cast<float>(config.H_Ki))
  {
    H_Ki_ = config.H_Ki;
    ROS_INFO("H_Ki new value %f", H_Ki_);
  }
  if (H_Kd_ != static_cast<float>(config.H_Kd))
  {
    H_Kd_ = config.H_Kd;
    ROS_INFO("H_Kd new value %f", H_Kd_);
  }

  if (M_Kp_ != static_cast<float>(config.M_Kp))
  {
    M_Kp_ = config.M_Kp;
    ROS_INFO("M_Kp new value %f", M_Kp_);
  }
  if (M_Ki_ != static_cast<float>(config.M_Ki))
  {
    M_Ki_ = config.M_Ki;
    ROS_INFO("M_Ki new value %f", M_Ki_);
  }
  if (M_Kd_ != static_cast<float>(config.M_Kd))
  {
    M_Kd_ = config.M_Kd;
    ROS_INFO("M_Kd new value %f", M_Kd_);
  }

  if (L_Kp_ != static_cast<float>(config.L_Kp))
  {
    L_Kp_ = config.L_Kp;
    ROS_INFO("L_Kp new value %f", L_Kp_);
  }
  if (L_Ki_ != static_cast<float>(config.L_Ki))
  {
    L_Ki_ = config.L_Ki;
    ROS_INFO("L_Ki new value %f", L_Ki_);
  }
  if (L_Kd_ != static_cast<float>(config.L_Kd))
  {
    L_Kd_ = config.L_Kd;
    ROS_INFO("L_Kd new value %f", L_Kd_);
  }
  if (LaneChange_Kp_ != static_cast<float>(config.LaneChange_Kp))
  {
    LaneChange_Kp_ = config.LaneChange_Kp;
    ROS_INFO("LaneChange_Kp new value %f", LaneChange_Kp_);
  }
  if (LaneChange_Ki_ != static_cast<float>(config.LaneChange_Ki))
  {
    LaneChange_Ki_ = config.LaneChange_Ki;
    ROS_INFO("LaneChange_Ki new value %f", LaneChange_Ki_);
  }
  if (LaneChange_Kd_ != static_cast<float>(config.LaneChange_Kd))
  {
    LaneChange_Kd_ = config.LaneChange_Kd;
    ROS_INFO("LaneChange_Kd new value %f", LaneChange_Kd_);
  }
  if (M_speed_ != static_cast<float>(config.M_speed))
  {
    M_speed_ = config.M_speed;
    ROS_INFO("M_speed new value %f", M_speed_);
  }
  if (H_speed_ != static_cast<float>(config.H_speed))
  {
    M_speed_ = config.H_speed;
    ROS_INFO("H_speed new value %f", H_speed_);
  }
}
