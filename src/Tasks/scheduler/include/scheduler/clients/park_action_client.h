/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/ 

#ifndef SCHEDULER_CLIENTS_PARK_ACTION_CLIENT_H
#define SCHEDULER_CLIENTS_PARK_ACTION_CLIENT_H

#include <scheduler/clients/client_interface.h>

#include <custom_msgs/parkAction.h>
#include <custom_msgs/enums.h>

#include <string>

class ParkClient : public ClientInterface
{
protected:
    actionlib::SimpleActionClient<custom_msgs::parkAction> ac_;
    custom_msgs::parkGoal goal_;
    int result_;
    ros::NodeHandle pnh_;
    int parking_steering_mode_;
    ros::ServiceClient muxDriveSelect_;
    ros::ServiceClient steeringModeSetAckermann_;
    ros::ServiceClient steeringModeSetParallel_;

    int sucessful_park_counter_;
    int park_atttempts_counter_;

public:
    explicit ParkClient(std::string name, const ros::NodeHandle &pnh);
    ~ParkClient();

    void setGoal(boost::any goal);
    bool waitForResult(float timeout);
    void cancelAction();
    bool waitForServer(float timeout);

    void doneCb(const actionlib::SimpleClientGoalState& state,
                    const custom_msgs::parkResultConstPtr& result);
    void activeCb();
    void feedbackCb(const custom_msgs::parkFeedbackConstPtr& feedback);
    bool getResult();
    void getActionResult(boost::any &result);
    void setParkSteeringMode();
    void prepareAction();
};


#endif  // SCHEDULER_CLIENTS_PARK_ACTION_CLIENT_H
