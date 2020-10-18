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
    pnh_(pnh),
    drive_steering_mode_(ACKERMANN),
    park_complete_(false)
{
    next_action_ = DRIVING;
    action_state_ = SELFIE_IDLE;
    result_flag_ = EMPTY;

    pnh_.getParam("drive_steering_mode", drive_steering_mode_);

    visionReset_ = nh_.serviceClient<std_srvs::Empty>("resetVision");
    resetLaneController_ = nh_.serviceClient<std_srvs::Empty>("resetLaneControl");
    muxDriveSelect_ = nh_.serviceClient<topic_tools::MuxSelect>("drive_multiplexer/select");
    steeringModeSetAckermann_ = nh_.serviceClient<std_srvs::Empty>("steering_ackerman");
    steeringModeSetParallel_ = nh_.serviceClient<std_srvs::Empty>("steering_parallel");
    steeringModeSetFrontAxis_ = nh_.serviceClient<std_srvs::Empty>("steering_front_axis");
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
        next_action_ = PARKING_SEARCH;
    else
        next_action_ = INTERSECTION;
}

void DriveClient::checkParkCounter(boost::any goal)
{
    const std::type_info &ti = goal.type();
    if (ti.name()[0] == 'i' && park_complete_ == false)
    {
        int i = boost::any_cast<int>(goal);
        if (i == PARKING_COMPLETE)
        {
            ROS_INFO("Park complete");
            next_action_ = DRIVING;
            park_complete_ = true;
        }
    }
}

void DriveClient::setGoal(boost::any goal)
{
    // check parking counter only for parking task
    if (drive_mode_ == false)
        checkParkCounter(goal);

    goal_.mode = drive_mode_;
    ac_.sendGoal(goal_, boost::bind(&DriveClient::doneCb, this, _1, _2),
                boost::bind(&DriveClient::activeCb, this),
                boost::bind(&DriveClient::feedbackCb, this, _1));
}

bool DriveClient::waitForResult(float timeout)
{
    return ac_.waitForResult(ros::Duration(timeout));
}

bool DriveClient::waitForServer(float timeout)
{
    result_flag_ = EMPTY;
    ROS_INFO("Wait for driving action server");
    return ac_.waitForServer(ros::Duration(timeout));
}

void DriveClient::doneCb(const actionlib::SimpleClientGoalState& state,
            const custom_msgs::drivingResultConstPtr& result)
{
    ROS_INFO("Finished drive in state [%s]", state.toString().c_str());
    ROS_INFO("drive result: %d",  result->event);

    if (state == State::ABORTED)
    {
        result_flag_ = ABORTED;
        std_srvs::Empty empty_msg;
        avoidingObstSetPassive_.call(empty_msg);
    }
    else
    {
        result_ = result->event;
        result_flag_ = SUCCESS;
    }
}

void DriveClient::activeCb()
{
    ROS_INFO("Drive acnext_action_tion server active");
}

void DriveClient::feedbackCb(const custom_msgs::drivingFeedbackConstPtr& feedback)
{
  action_state_ = (program_state)feedback->action_status;
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

void DriveClient::setDriveSteeringMode()
{
    std_srvs::Empty empty_msg;
    if (drive_steering_mode_ == PARALLEL)
        steeringModeSetParallel_.call(empty_msg);
    else if (drive_steering_mode_ == ACKERMANN)
        steeringModeSetAckermann_.call(empty_msg);
    else if (drive_steering_mode_ == FRONT_AXIS)
        steeringModeSetFrontAxis_.call(empty_msg);
}

void DriveClient::prepareAction()
{
    std_srvs::Empty empty_msg;
    visionReset_.call(empty_msg);
    resetLaneController_.call(empty_msg);
    setDriveSteeringMode();

    topic_tools::MuxSelect topic_sel;
    topic_sel.request.topic = "drive/lane_control";
    muxDriveSelect_.call(topic_sel);

    if (drive_mode_)
        avoidingObstSetActive_.call(empty_msg);  // intersection task
    else
        avoidingObstSetPassive_.call(empty_msg);  // parking task

    ROS_INFO("Prepare drive - vision reset, reset Lane control, MuxSelect drive/lane_control,"
             "set drive steering mode %d, avoinding obst set %d", drive_steering_mode_, drive_mode_);
}
