/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/ 

#ifndef SCHEDULER_CLIENTS_CLIENT_INTERFACE_H
#define SCHEDULER_CLIENTS_CLIENT_INTERFACE_H

#include <boost/any.hpp>

#include <geometry_msgs/Point.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <custom_msgs/enums.h>

class ClientInterface
{
protected:
    ros::NodeHandle nh_;
    program_state action_state_;
    client_goal_state result_flag_;
    action next_action_;

public:
    virtual ~ClientInterface() = 0;
    virtual bool waitForServer(float timeout) = 0;
    virtual void setGoal(boost::any goal) = 0;
    virtual bool waitForResult(float timeout) = 0;
    virtual void cancelAction() = 0;
    virtual void getActionResult(boost::any &result) = 0;
    virtual void prepareAction() = 0;

    client_goal_state getClientGoalState() {return result_flag_;}
    action getNextAction() {return next_action_;}
    program_state getActionState()
    {
        if (action_state_ != SELFIE_IDLE)
            return action_state_;
    }
};
#endif  // SCHEDULER_CLIENTS_CLIENT_INTERFACE_H
