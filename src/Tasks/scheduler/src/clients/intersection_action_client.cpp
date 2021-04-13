/**
 *Copyright ( c ) 2019, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#include <scheduler/clients/intersection_action_client.h>
#include <std_srvs/Empty.h>

#include <string>

IntersectionClient::IntersectionClient(std::string name) : ac_(name, true)
{
  next_action_ = selfie::FREE_DRIVE;
  avoidingObstSetPassive_ = nh_.serviceClient<std_srvs::Empty>("avoiding_obst_set_passive");
  resetLaneController_ = nh_.serviceClient<std_srvs::Empty>("resetLaneControl");
}
IntersectionClient::~IntersectionClient()
{
}

void IntersectionClient::setGoal(boost::any goal)
{
  ac_.sendGoal(
      goal_, boost::bind(&IntersectionClient::doneCb, this, _1, _2), boost::bind(&IntersectionClient::activeCb, this));
  goal_state_flag_ = SENT;
}

bool IntersectionClient::waitForResult(float timeout)
{
  return ac_.waitForResult(ros::Duration(timeout));
}

bool IntersectionClient::waitForServer(float timeout)
{
  goal_state_flag_ = NOT_SEND;
  ROS_INFO("Wait for itersection action server");
  return ac_.waitForServer(ros::Duration(timeout));
}

void IntersectionClient::doneCb(const actionlib::SimpleClientGoalState& state,
                                const custom_msgs::intersectionResultConstPtr& result)
{
  ROS_INFO("Finished itersection in state [%s]", state.toString().c_str());
  ROS_INFO("itersection result: %d", result->done);
  if (state == actionlib::SimpleClientGoalState::StateEnum::ABORTED)
  {
    goal_state_flag_ = ABORTED;
  }
  else if (state == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED)
  {
    result_ = result->done;
    goal_state_flag_ = SUCCESS;
  }
}

void IntersectionClient::activeCb()
{
  ROS_INFO("itersection next_action_tion server active");
}

void IntersectionClient::cancelAction()
{
  ROS_INFO("itersection cancel action");
  ac_.cancelAllGoals();
}

void IntersectionClient::getActionResult(boost::any& result)
{
  // result = result_;
}

void IntersectionClient::prepareAction()
{
  goal_state_flag_ = NOT_SEND;
  std_srvs::Empty empty_msg;
  resetLaneController_.call(empty_msg);
  avoidingObstSetPassive_.call(empty_msg);
  ROS_INFO("Prepare intersection - call reset Lane control and avoiding obst passive");
}
