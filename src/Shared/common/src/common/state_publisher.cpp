/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#include <common/state_publisher.h>
#include <string>

StatePublisher::StatePublisher(const std::string& topic_name, float pub_freq) : nh_("~")
{
    publisher_ = nh_.advertise<std_msgs::Int8>(topic_name, 1);
    state_.data = 0;  // UNINITIALIZED
    timer_ = nh_.createTimer(ros::Duration(pub_freq), &StatePublisher::timerCallback, this);
    timer_.start();
    ROS_INFO("State publisher timer started on %s topic", topic_name.c_str());
}

StatePublisher::StatePublisher(const std::string& topic_name) : nh_("~")
{
    publisher_ = nh_.advertise<std_msgs::Int8>(topic_name, 1, true);
    state_.data = 0;  // UNINITIALIZED
}

void StatePublisher::updateState(int8_t new_state)
{
    state_.data = new_state;
    sendState();
}

void StatePublisher::updateState(std_msgs::Int8 new_state)
{
    state_ = new_state;
    sendState();
}

void StatePublisher::timerCallback(const ros::TimerEvent& event)
{
    sendState();
}

void StatePublisher::sendState()
{
    publisher_.publish(state_);
}
