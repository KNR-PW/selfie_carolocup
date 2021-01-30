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
  set_ackermann_settings_srv_ = nh_.advertiseService("/pidTuner/setAckerman", &PidTuner::setAckermannSettingsCb, this);
  set_default_settings_srv_ = nh_.advertiseService("/pidTuner/setDefault", &PidTuner::setDefaultSettingsCb, this);
  use_ackermann_settings_ = false;

  pnh_.getParam("pid_tuner_disabled", pid_tuner_disabled);

  pnh_.getParam("L_Kp", L_Kp);
  pnh_.getParam("L_Ki", L_Ki);
  pnh_.getParam("L_Kd", L_Kd);

  pnh_.getParam("M_Kp", M_Kp);
  pnh_.getParam("M_Ki", M_Ki);
  pnh_.getParam("M_Kd", M_Kd);

  pnh_.getParam("H_Kp", H_Kp);
  pnh_.getParam("H_Ki", H_Ki);
  pnh_.getParam("H_Kd", H_Kd);

  pnh_.getParam("A_Kp", A_Kp);
  pnh_.getParam("A_Ki", A_Kp);
  pnh_.getParam("A_Kd", A_Kp);

  pnh_.getParam("H_speed", H_speed);
  pnh_.getParam("M_speed", M_speed);
  pnh_.getParam("speed_change_treshold", speed_change_treshold);
}

void PidTuner::speedCallback(const custom_msgs::Motion& msg)
{
  if ((abs(msg.speed_linear - act_speed_) < speed_change_treshold) || use_ackermann_settings_ || pid_tuner_disabled)
  {
    return;
  }

  act_speed_ = msg.speed_linear;
  if (act_speed_ < M_speed)
  {
    setKp(L_Kp);
    setKd(L_Kd);
    setKi(L_Ki);
  }
  else if (act_speed_ < H_speed)
  {
    setKp(M_Kp);
    setKd(M_Kd);
    setKi(M_Ki);
  }
  else
  {
    setKp(H_Kp);
    setKd(H_Kd);
    setKi(H_Ki);
  }
}

bool PidTuner::setDefaultSettingsCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  use_ackermann_settings_ = false;
  act_speed_ = 0.0; // to be sure that speed callback will act immediately
  ROS_INFO("Use default PID settings");
  return true;
}

bool PidTuner::setAckermannSettingsCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  use_ackermann_settings_ = true;
  ROS_INFO("Use ackermann PID settings");
  setKp(A_Kp);
  setKd(A_Kd);
  setKi(A_Ki);
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

void PidTuner::reconfigureCB(const pid_carrot_follower::PIDTunerConfig& config, uint32_t level)
{
  if(pid_tuner_disabled != static_cast<bool>(config.pid_tuner_disabled))
  {
    pid_tuner_disabled = config.pid_tuner_disabled;
    if(pid_tuner_disabled)
      ROS_INFO("PID tuner disabled");
    else
      ROS_INFO("PID tuner enabled");
  }
  if (H_Kp != static_cast<float>(config.H_Kp))
  {
    H_Kp = config.H_Kp;
    ROS_INFO("H_Kp new value %f", H_Kp);
  }
  if (H_Ki != static_cast<float>(config.H_Ki))
  {
    H_Ki = config.H_Ki;
    ROS_INFO("H_Ki new value %f", H_Ki);
  }
  if (H_Kd != static_cast<float>(config.H_Kd))
  {
    H_Kd = config.H_Kd;
    ROS_INFO("H_Kd new value %f", H_Kd);
  }

  if (M_Kp != static_cast<float>(config.M_Kp))
  {
    M_Kp = config.M_Kp;
    ROS_INFO("M_Kp new value %f", M_Kp);
  }
  if (M_Ki != static_cast<float>(config.M_Ki))
  {
    M_Ki = config.M_Ki;
    ROS_INFO("M_Ki new value %f", M_Ki);
  }
  if (M_Kd != static_cast<float>(config.M_Kd))
  {
    M_Kd = config.M_Kd;
    ROS_INFO("M_Kd new value %f", M_Kd);
  }

  if (L_Kp != static_cast<float>(config.L_Kp))
  {
    L_Kp = config.L_Kp;
    ROS_INFO("L_Kp new value %f", L_Kp);
  }
  if (L_Ki != static_cast<float>(config.L_Ki))
  {
    L_Ki = config.L_Ki;
    ROS_INFO("L_Ki new value %f", L_Ki);
  }
  if (L_Kd != static_cast<float>(config.L_Kd))
  {
    L_Kd = config.L_Kd;
    ROS_INFO("L_Kd new value %f", L_Kd);
  }
  if (A_Kp != static_cast<float>(config.A_Kp))
  {
    A_Kp = config.A_Kp;
    ROS_INFO("A_Kp new value %f", A_Kp);
  }
  if (A_Ki != static_cast<float>(config.A_Ki))
  {
    A_Ki = config.A_Ki;
    ROS_INFO("A_Ki new value %f", A_Ki);
  }
  if (A_Kd != static_cast<float>(config.A_Kd))
  {
    A_Kd = config.A_Kd;
    ROS_INFO("A_Kd new value %f", A_Kd);
  }
  if (M_speed != static_cast<float>(config.M_speed))
  {
    M_speed = config.M_speed;
    ROS_INFO("M_speed new value %f", M_speed);
  }
  if (H_speed != static_cast<float>(config.H_speed))
  {
    M_speed = config.H_speed;
    ROS_INFO("H_speed new value %f", H_speed);
  }
}
