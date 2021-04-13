/**
 *Copyright ( c ) 2019, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef SCHEDULER_CLIENTS_SEARCH_ACTION_CLIENT_H
#define SCHEDULER_CLIENTS_SEARCH_ACTION_CLIENT_H

#include <scheduler/clients/client_interface.h>

#include <custom_msgs/searchAction.h>

#include <string>

class SearchClient : public ClientInterface
{
protected:
  actionlib::SimpleActionClient<custom_msgs::searchAction> ac_;
  custom_msgs::searchGoal goal_;
  custom_msgs::Box2D result_;

public:
  explicit SearchClient(std::string name);
  ~SearchClient();

  void setGoal(boost::any goal);
  bool waitForResult(float timeout);
  void cancelAction();
  bool waitForServer(float timeout);

  void doneCb(const actionlib::SimpleClientGoalState& state, const custom_msgs::searchResultConstPtr& result);
  void activeCb();
  void getActionResult(boost::any& result);
  void prepareAction();
};

#endif  // SCHEDULER_CLIENTS_SEARCH_ACTION_CLIENT_H
