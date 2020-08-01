/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/ 

#ifndef SCHEDULER_CLIENTS_DRIVE_ACTION_CLIENT_H
#define SCHEDULER_CLIENTS_DRIVE_ACTION_CLIENT_H

#include <scheduler/clients/client_interface.h>

#include <custom_msgs/drivingAction.h>
#include <custom_msgs/enums.h>

#include <string>

class DriveClient : public ClientInterface
{
protected:
    actionlib::SimpleActionClient<custom_msgs::drivingAction> ac_;
    custom_msgs::drivingGoal goal_;
    bool result_;
    bool drive_mode_;
    int drive_steering_mode_;
    bool park_complete_;

    ros::ServiceClient visionReset_;
    ros::ServiceClient resetLaneController_;
    ros::ServiceClient muxDriveSelect_;
    ros::ServiceClient steeringModeSetAckermann_;
    ros::ServiceClient steeringModeSetParallel_;
    ros::ServiceClient steeringModeSetFrontAxis_;
    ros::ServiceClient avoidingObstSetPassive_;
    ros::ServiceClient avoidingObstSetActive_;

    ros::NodeHandle pnh_;
    void checkParkCounter(boost::any goal);

public:
    DriveClient(std::string name, const ros::NodeHandle &pnh);
    ~DriveClient();

    void setScenario(bool drive_mode);
    void setGoal(boost::any goal);
    bool waitForResult(float timeout);
    void cancelAction();
    bool waitForServer(float timeout);

    void doneCb(const actionlib::SimpleClientGoalState& state,
                    const custom_msgs::drivingResultConstPtr& result);
    void activeCb();
    void feedbackCb(const custom_msgs::drivingFeedbackConstPtr& feedback);
    void getActionResult(boost::any &result);
    void setDriveSteeringMode();
    void prepareAction();
};


#endif  // SCHEDULER_CLIENTS_DRIVE_ACTION_CLIENT_H
