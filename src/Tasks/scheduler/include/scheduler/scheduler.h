/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/ 

#ifndef SCHEDULER_SCHEDULER_H
#define SCHEDULER_SCHEDULER_H

#include <boost/any.hpp>
#include <std_msgs/UInt8.h>

#include <custom_msgs/enums.h>
#include <scheduler/clients/client_interface.h>

#include <map>

/*
Params:
- start action + default arguments (pressed button, detected parking spot)
- starting_distance
- parking_spot
-
*/

class Scheduler
{
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // params
    int begin_action_;
    float start_distance_;
    float parking_spot_;

    program_state current_car_state_;
    program_state previous_car_state_;
    action current_action_state_;

    std::map<action, ClientInterface*> clients_;
    std::map<action, boost::any> action_args_;
    ClientInterface *current_client_ptr_;

    ros::Subscriber switchState_;
    void switchStateCallback(const std_msgs::UInt8ConstPtr &msg);
    rc_state previousRcState_;
    rc_state currentRcState_;

    template <typename T> bool checkCurrentClientType();
    void stateMachine();
    void startAction(action action_to_set);
    void startNextAction();
    void stopAction();

public:
    Scheduler();
    ~Scheduler();

    void init();

    void loop();

    void actionSetup();  // begin action
    void logFeedback();
    void feedbackStateMachine();

    void setupActionClients(bool button_pressed);
    void waitForStart();

    action getBeginAction();  // get first action from argument
    void shiftAction();

    void actionStateMachine();
    int checkIfActionFinished();
};

#endif  // SCHEDULER_SCHEDULER_H
