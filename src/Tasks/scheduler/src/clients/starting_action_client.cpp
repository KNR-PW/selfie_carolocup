/**
 *Copyright ( c ) 2019, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#include <scheduler/clients/starting_action_client.h>

#include <topic_tools/MuxSelect.h>
#include <string>

StartingProcedureClient::StartingProcedureClient(std::string name) : ac_(name, true)
{
  next_action_ = selfie::FREE_DRIVE;

  muxDriveSelect_ = nh_.serviceClient<topic_tools::MuxSelect>("drive_multiplexer/select");
}
StartingProcedureClient::~StartingProcedureClient()
{
}

void StartingProcedureClient::setGoal(boost::any goal)
{
  float distance;

  try
  {
    distance = boost::any_cast<float>(goal);
  }
  catch (boost::bad_any_cast& e)
  {
    ROS_ERROR("bad casting %s", e.what());
    return;
  }
  goal_.distance = distance;
  ac_.sendGoal(goal_,
               boost::bind(&StartingProcedureClient::doneCb, this, _1, _2),
               boost::bind(&StartingProcedureClient::activeCb, this));
  goal_state_flag_ = SENT;
}

bool StartingProcedureClient::waitForResult(float timeout)
{
  return ac_.waitForResult(ros::Duration(timeout));
}

bool StartingProcedureClient::waitForServer(float timeout)
{
  goal_state_flag_ = NOT_SEND;
  ROS_INFO("Wait for starting procedure action server");
  return ac_.waitForServer(ros::Duration(timeout));
}

void StartingProcedureClient::doneCb(const actionlib::SimpleClientGoalState& state,
                                     const custom_msgs::startingResultConstPtr& result)
{
  ROS_INFO("Finished starting in state [%s]", state.toString().c_str());
  if (state == actionlib::SimpleClientGoalState::StateEnum::ABORTED)
  {
    goal_state_flag_ = ABORTED;
  }
  else
  {
    ROS_INFO("starting result: %i", result->drive_mode);
    result_ = result->drive_mode;
    goal_state_flag_ = SUCCESS;
  }
}

void StartingProcedureClient::activeCb()
{
  ROS_INFO("Starting procedure server active");
}

void StartingProcedureClient::cancelAction()
{
  ac_.cancelAllGoals();
}

void StartingProcedureClient::getActionResult(boost::any& result)
{
  result = result_;
}

void StartingProcedureClient::prepareAction()
{
  goal_state_flag_ = NOT_SEND;
  topic_tools::MuxSelect topic_sel;
  topic_sel.request.topic = "drive/starting_procedure";
  muxDriveSelect_.call(topic_sel);

  ROS_INFO("Prepare starting - MuxSelect drive/starting_procedure");
}
