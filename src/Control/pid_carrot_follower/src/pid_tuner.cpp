#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <pid_carrot_follower/pid_tuner.h>

#include <pid/PidConfig.h>
#include <dynamic_reconfigure/Config.h>

PidTuner::PidTuner() 
: pnh_("~")
, dr_server_CB_(boost::bind(&PidTuner::reconfigureCB, this, _1, _2))
{
  dr_server_.setCallback(dr_server_CB_);

  sub_motion_ = nh_.subscribe("/selfie_out/motion", 50, &PidTuner::speedCallback, this);

  pnh_.getParam("L_Kp", L_Kp);
  pnh_.getParam("L_Ki", L_Ki);
  pnh_.getParam("L_Kd", L_Kd);

  pnh_.getParam("M_Kp", M_Kp);
  pnh_.getParam("M_Ki", M_Ki);
  pnh_.getParam("M_Kd", M_Kd);

  pnh_.getParam("H_Kp", H_Kp);
  pnh_.getParam("H_Ki", H_Ki);
  pnh_.getParam("H_Kd", H_Kd);

  pnh_.getParam("H_speed", H_speed);
  pnh_.getParam("M_speed", M_speed);

}

void PidTuner::speedCallback(const custom_msgs::Motion &msg)
{
  if (abs(msg.speed_linear - act_speed_) < 0.1) // Add parameter
  {
    act_speed_ = msg.speed_linear;
    return;
  }

  act_speed_ = msg.speed_linear;
  if(act_speed_ < M_speed)
  {
    setKp(L_Kp);
    setKd(L_Kd);
    setKi(L_Ki);
  }
  else if(act_speed_ < H_speed)
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

  pnh_.setParam("/pid_controller/Kd",Kd);
  pnh_.setParam("/pid_controller/Kd_scale",scale);
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

  pnh_.setParam("/pid_controller/Kp",Kp);
  pnh_.setParam("/pid_controller/Kp_scale",scale);
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

  pnh_.setParam("/pid_controller/Ki",Ki);
  pnh_.setParam("/pid_controller/Ki_scale",scale);
}

void PidTuner::reconfigureCB(pid_carrot_follower::PIDTunerConfig& config, uint32_t level)
{
  if(H_Kp != (float)config.H_Kp)
  {
    H_Kp = config.H_Kp;
    ROS_INFO("H_Kp new value %f",H_Kp);
  }
  if(H_Ki != (float)config.H_Ki)
  {
    H_Ki = config.H_Ki;
    ROS_INFO("H_Ki new value %f",H_Ki);
  }
  if(H_Kd != (float)config.H_Kd)
  {
    H_Kd = config.H_Kd;
    ROS_INFO("H_Kd new value %f", H_Kd);
  }

  if(M_Kp != (float)config.M_Kp)
  {
    M_Kp = config.M_Kp;
    ROS_INFO("M_Kp new value %f",M_Kp);
  }
  if(M_Ki != (float)config.M_Ki)
  {
    M_Ki = config.M_Ki;
    ROS_INFO("M_Ki new value %f",M_Ki);
  }
  if(M_Kd != (float)config.M_Kd)
  {
    M_Kd = config.M_Kd;
    ROS_INFO("M_Kd new value %f", M_Kd);
  }

  if(L_Kp != (float)config.L_Kp)
  {
    L_Kp = config.L_Kp;
    ROS_INFO("L_Kp new value %f",L_Kp);
  }
  if(L_Ki != (float)config.L_Ki)
  {
    L_Ki = config.L_Ki;
    ROS_INFO("L_Ki new value %f",L_Ki);
  }
  if(L_Kd != (float)config.L_Kd)
  {
    L_Kd = config.L_Kd;
    ROS_INFO("L_Kd new value %f", L_Kd);
  }
  if(M_speed != (float)config.M_speed)
  {
    M_speed = config.M_speed;
    ROS_INFO("M_speed new value %f",M_speed);
  }
  if(H_speed != (float)config.H_speed)
  {
    M_speed = config.H_speed;
    ROS_INFO("H_speed new value %f",H_speed);
  }
}