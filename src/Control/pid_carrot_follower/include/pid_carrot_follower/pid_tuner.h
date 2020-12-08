/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef PARK_PARK_H
#define PARK_PARK_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pid_carrot_follower/PIDTunerConfig.h>


class PidTuner 
{
    ros::NodeHandle pnh_;
    ros::NodeHandle nh_;
    // variables used for changing settings of PID
    dynamic_reconfigure::ReconfigureRequest srv_req_;
    dynamic_reconfigure::ReconfigureResponse srv_resp_;
    dynamic_reconfigure::DoubleParameter double_param_;
    dynamic_reconfigure::Config conf_;

    dynamic_reconfigure::Server<pid_carrot_follower::PIDTunerConfig> dr_server_;
    dynamic_reconfigure::Server<pid_carrot_follower::PIDTunerConfig>::CallbackType dr_server_CB_;

    bool running = true;
    bool change_switch = false;

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

public:
    PidTuner();
    void setKd(float Kd);
    void setKp(float Kp);
    void setKi(float Ki);
private: 
    void reconfigureCB(pid_carrot_follower::PIDTunerConfig& config, uint32_t level);

};
#endif //PARK_PARK_H