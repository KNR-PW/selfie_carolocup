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

#include <custom_msgs/action_enum.h>

typedef enum GoalState
{
    NOT_SEND,
    SENT,
    ABORTED,
    SUCCESS,
}
GoalState;

class ClientInterface
{
protected:
    ros::NodeHandle nh_;
    GoalState goal_state_flag_{NOT_SEND};
    selfie::EnumAction next_action_;

public:
    virtual ~ClientInterface() = 0;
    virtual bool waitForServer(float timeout) = 0;
    virtual void setGoal(boost::any goal) = 0;
    virtual bool waitForResult(float timeout) = 0;
    virtual void cancelAction() = 0;
    virtual void getActionResult(boost::any &result) = 0;
    virtual void prepareAction() = 0;

    GoalState getGoalState() {return goal_state_flag_;}
    selfie::EnumAction getNextAction() {return next_action_;}
};
#endif  // SCHEDULER_CLIENTS_CLIENT_INTERFACE_H
