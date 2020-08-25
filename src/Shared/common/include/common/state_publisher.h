/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef COMMON_STATE_PUBLISHER_H
#define COMMON_STATE_PUBLISHER_H

#include <ros/ros.h>
#include <ros/timer.h>
#include <string>
#include <std_msgs/Int8.h>

class StatePublisher
{
    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::Publisher publisher_;
    std_msgs::Int8 state_;

public:
    StatePublisher(const std::string& topic_name, float pub_freq);
    void updateState(int8_t new_state);
    void updateState(std_msgs::Int8 new_state);

private:
    void sendState();
    void timerCallback(const ros::TimerEvent& event);
};
#endif  // COMMON_STATE_PUBLISHER_H
