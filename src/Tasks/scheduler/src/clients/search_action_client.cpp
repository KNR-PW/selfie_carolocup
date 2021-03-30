/**
 *Copyright ( c ) 2019, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#include <scheduler/clients/search_action_client.h>

#include <string>

SearchClient::SearchClient(std::string name) : ac_(name, true)
{
  next_action_ = selfie::PARK;
}

SearchClient::~SearchClient()
{
}

void SearchClient::setGoal(boost::any goal)
{
  float parking_spot;

  try
  {
    parking_spot = boost::any_cast<float>(goal);
  }
  catch (boost::bad_any_cast& e)
  {
    ROS_ERROR("bad casting %s", e.what());
    return;
  }
  goal_.min_spot_lenght = parking_spot;
  ac_.sendGoal(goal_, boost::bind(&SearchClient::doneCb, this, _1, _2), boost::bind(&SearchClient::activeCb, this));
  goal_state_flag_ = SENT;
}
bool SearchClient::waitForResult(float timeout)
{
  return ac_.waitForResult(ros::Duration(timeout));
}

bool SearchClient::waitForServer(float timeout)
{
  goal_state_flag_ = NOT_SEND;
  ROS_INFO("Wait for search action server");
  return ac_.waitForServer(ros::Duration(timeout));
}

void SearchClient::doneCb(const actionlib::SimpleClientGoalState& state,
                          const custom_msgs::searchResultConstPtr& result)
{
  ROS_INFO("Finished search in state [%s]", state.toString().c_str());

  if (state == actionlib::SimpleClientGoalState::StateEnum::ABORTED)
  {
    goal_state_flag_ = ABORTED;
  }
  else
  {
    result_ = result->parking_spot;
    goal_state_flag_ = SUCCESS;
  }
}

void SearchClient::activeCb()
{
  ROS_INFO("Search action server active");
}

void SearchClient::cancelAction()
{
  ac_.cancelAllGoals();
}

void SearchClient::getActionResult(boost::any& goal)
{
  goal = result_;
}

void SearchClient::prepareAction()
{
  goal_state_flag_ = NOT_SEND;
}
