/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/ 

#include <scheduler/clients/park_action_client.h>
#include <std_srvs/Empty.h>

#include <topic_tools/MuxSelect.h>
#include <string>

ParkClient::ParkClient(std::string name, const ros::NodeHandle &pnh):
    ac_(name, true),
    pnh_(pnh),
    result_(0)
{
    next_action_ = selfie::FREE_DRIVE;
    muxDriveSelect_ = nh_.serviceClient<topic_tools::MuxSelect>("drive_multiplexer/select");
}

ParkClient::~ParkClient()
{
}

void ParkClient::setGoal(boost::any goal)
{
    custom_msgs::Box2D parking_spot;

    try
    {
        parking_spot = boost::any_cast<custom_msgs::Box2D>(goal);
    }
    catch (boost::bad_any_cast &e)
    {
        ROS_ERROR("bad casting %s", e.what());
        return;
    }
    goal_.parking_spot = parking_spot;
    ac_.sendGoal(goal_, boost::bind(&ParkClient::doneCb, this, _1, _2),
                boost::bind(&ParkClient::activeCb, this));
    goal_state_flag_ = SENT;
}

bool ParkClient::waitForResult(float timeout)
{
    return ac_.waitForResult(ros::Duration(timeout));
}

bool ParkClient::waitForServer(float timeout)
{
    goal_state_flag_ = NOT_SEND;
    ROS_INFO("Wait for park action server");
    return ac_.waitForServer(ros::Duration(timeout));
}

void ParkClient::doneCb(const actionlib::SimpleClientGoalState& state,
                        const custom_msgs::parkResultConstPtr& result)
{
    ROS_INFO("Finished park in state [%s]", state.toString().c_str());
    if (state == actionlib::SimpleClientGoalState::StateEnum::ABORTED)
    {
        goal_state_flag_ = ABORTED;
    }
    else
    {
        goal_state_flag_ = SUCCESS;
    }
}

void ParkClient::activeCb()
{
    ROS_INFO("Park server active");
}

void ParkClient::cancelAction()
{
  ac_.cancelAllGoals();
}

void ParkClient::getActionResult(boost::any &result)
{
    result = result_;
}

void ParkClient::prepareAction()
{
    goal_state_flag_ = NOT_SEND;
    std_srvs::Empty empty_msg;

    topic_tools::MuxSelect topic_sel;
    topic_sel.request.topic = "drive/park";
    muxDriveSelect_.call(topic_sel);

    ROS_INFO("Prepare park - MuxSelect drive/park");
}
