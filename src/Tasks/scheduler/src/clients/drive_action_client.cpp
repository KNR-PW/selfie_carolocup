/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/ 

#include <scheduler/clients/drive_action_client.h>

#include <std_srvs/Empty.h>
#include <topic_tools/MuxSelect.h>
#include <typeinfo>
#include <string>

DriveClient::DriveClient(std::string name, const ros::NodeHandle &pnh):
    ac_(name, true),
    pnh_(pnh)
{
    next_action_ = selfie::FREE_DRIVE;

    visionReset_ = nh_.serviceClient<std_srvs::Empty>("resetVision");
    resetLaneController_ = nh_.serviceClient<std_srvs::Empty>("resetLaneControl");
    muxDriveSelect_ = nh_.serviceClient<topic_tools::MuxSelect>("drive_multiplexer/select");
    avoidingObstSetPassive_ = nh_.serviceClient<std_srvs::Empty>("avoiding_obst_set_passive");
    avoidingObstSetActive_ = nh_.serviceClient<std_srvs::Empty>("avoiding_obst_set_active");
}

DriveClient::~DriveClient()
{
}

void DriveClient::setScenario(bool drive_mode)
{
    drive_mode_ = drive_mode;

    if (drive_mode_ == false)
        next_action_ = selfie::PARKING_SPOT_SEARCH;
    else
        next_action_ = selfie::INTERSECTION_STOP;
}

void DriveClient::removeNextAction()
{
    next_action_ = selfie::FREE_DRIVE;
}

void DriveClient::setGoal(boost::any goal)
{
    goal_.mode = drive_mode_;
    ac_.sendGoal(goal_, boost::bind(&DriveClient::doneCb, this, _1, _2),
                boost::bind(&DriveClient::activeCb, this));
    goal_state_flag_ = SENT;
}

bool DriveClient::waitForResult(float timeout)
{
    return ac_.waitForResult(ros::Duration(timeout));
}

bool DriveClient::waitForServer(float timeout)
{
    goal_state_flag_ = NOT_SEND;
    ROS_INFO("Wait for driving action server");
    return ac_.waitForServer(ros::Duration(timeout));
}

void DriveClient::doneCb(const actionlib::SimpleClientGoalState& state,
                         const custom_msgs::drivingResultConstPtr& result)
{
    ROS_INFO("Finished drive in state [%s]", state.toString().c_str());
    ROS_INFO("drive result: %d",  result->event);

    if (state == actionlib::SimpleClientGoalState::StateEnum::ABORTED)
    {
        goal_state_flag_ = ABORTED;
        std_srvs::Empty empty_msg;
        avoidingObstSetPassive_.call(empty_msg);
    }
    else
    {
        result_ = result->event;
        goal_state_flag_ = SUCCESS;
    }
}

void DriveClient::activeCb()
{
    ROS_INFO("Drive action server active");
}

void DriveClient::cancelAction()
{
  ROS_INFO("Drive cancel action");
  ac_.cancelAllGoals();
}

void DriveClient::getActionResult(boost::any &result)
{
    // result = result_;
}

void DriveClient::prepareAction()
{
    goal_state_flag_ = NOT_SEND;
    std_srvs::Empty empty_msg;
    visionReset_.call(empty_msg);
    resetLaneController_.call(empty_msg);

    topic_tools::MuxSelect topic_sel;
    topic_sel.request.topic = "drive/lane_control";
    muxDriveSelect_.call(topic_sel);

    if (drive_mode_)
        avoidingObstSetActive_.call(empty_msg);  // intersection task
    else
        avoidingObstSetPassive_.call(empty_msg);  // parking task

    ROS_INFO("Prepare drive - vision reset, reset Lane control, MuxSelect drive/lane_control,"
             "avoinding obst set %d", drive_mode_);
}
