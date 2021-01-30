/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef PID_CARROT_FOLLOWER_PID_TUNER_H
#define PID_CARROT_FOLLOWER_PID_TUNER_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pid_carrot_follower/PIDTunerConfig.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include "custom_msgs/Motion.h"

class PidTuner
{
  ros::NodeHandle pnh_;
  ros::NodeHandle nh_;

  ros::Subscriber sub_motion_;
  // variables used for changing settings of PID
  dynamic_reconfigure::ReconfigureRequest srv_req_;
  dynamic_reconfigure::ReconfigureResponse srv_resp_;
  dynamic_reconfigure::DoubleParameter double_param_;
  dynamic_reconfigure::Config conf_;

  dynamic_reconfigure::Server<pid_carrot_follower::PIDTunerConfig> dr_server_;
  dynamic_reconfigure::Server<pid_carrot_follower::PIDTunerConfig>::CallbackType dr_server_CB_;

    ros::ServiceServer set_ackermann_settings_srv_;
    ros::ServiceServer set_default_settings_srv_;
    bool use_ackermann_settings_;

    bool pid_tuner_disabled;

  float kp_base = 1.0;
  float speed_base = 1.0;
  float coeff = 0.5;
  float deadzone = 0.1;
  float kp_base_scale = 1;

  float L_Kp;
  float L_Ki;
  float L_Kd;

  float M_Kp;
  float M_Ki;
  float M_Kd;

  float H_Kp;
  float H_Ki;
  float H_Kd;

    float A_Kp;
    float A_Ki;
    float A_Kd;

    float M_speed;
    float H_speed;

  float speed_change_treshold;
  float act_speed_;

public:
  PidTuner();
  void setKd(float Kd);
  void setKp(float Kp);
  void setKi(float Ki);

private:
    void reconfigureCB(pid_carrot_follower::PIDTunerConfig& config, uint32_t level);
    void speedCallback(const custom_msgs::Motion &msg);
    bool setDefaultSettingsCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool setAckermannSettingsCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
};
#endif  // PID_CARROT_FOLLOWER_PID_TUNER_H
