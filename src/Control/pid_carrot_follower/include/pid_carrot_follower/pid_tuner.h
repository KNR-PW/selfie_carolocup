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

  ros::ServiceServer set_lane_change_pid_settings_srv_;
  ros::ServiceServer set_default_pid_settings_srv_;

  bool use_lane_change_pid_settings_;
  bool pid_tuner_disabled_;

  float L_Kp_;
  float L_Ki_;
  float L_Kd_;

  float M_Kp_;
  float M_Ki_;
  float M_Kd_;

  float H_Kp_;
  float H_Ki_;
  float H_Kd_;

  float LaneChange_Kp_;
  float LaneChange_Ki_;
  float LaneChange_Kd_;

  float M_speed_;
  float H_speed_;

  float speed_change_treshold_;
  float act_speed_;

public:
  PidTuner();
  void setKd(float Kd);
  void setKp(float Kp);
  void setKi(float Ki);

private:
  void reconfigureCB(pid_carrot_follower::PIDTunerConfig& config, uint32_t level);
  void speedCallback(const custom_msgs::Motion& msg);
  bool setDefaultPidSettingsCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool setLaneChangePidSettingsCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
};
#endif  // PID_CARROT_FOLLOWER_PID_TUNER_H
