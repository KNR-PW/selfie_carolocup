/**
 *Copyright ( c ) 2019, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef SCHEDULER_CLIENTS_STARTING_ACTION_CLIENT_H
#define SCHEDULER_CLIENTS_STARTING_ACTION_CLIENT_H

#include <scheduler/clients/client_interface.h>

#include <custom_msgs/startingAction.h>

#include <string>

class StartingProcedureClient : public ClientInterface
{
protected:
  actionlib::SimpleActionClient<custom_msgs::startingAction> ac_;
  custom_msgs::startingGoal goal_;
  bool result_;

  ros::ServiceClient muxDriveSelect_;

public:
  explicit StartingProcedureClient(std::string name);
  ~StartingProcedureClient();

  void setGoal(boost::any goal);
  bool waitForResult(float timeout);
  void cancelAction();
  bool waitForServer(float timeout);

  void doneCb(const actionlib::SimpleClientGoalState& state, const custom_msgs::startingResultConstPtr& result);
  void activeCb();
  void getActionResult(boost::any& result);
  void prepareAction();
};

#endif  // SCHEDULER_CLIENTS_STARTING_ACTION_CLIENT_H
