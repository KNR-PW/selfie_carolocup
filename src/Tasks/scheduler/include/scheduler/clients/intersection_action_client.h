/**
 *Copyright ( c ) 2019, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef SCHEDULER_CLIENTS_INTERSECTION_ACTION_CLIENT_H
#define SCHEDULER_CLIENTS_INTERSECTION_ACTION_CLIENT_H

#include <scheduler/clients/client_interface.h>

#include <custom_msgs/intersectionAction.h>

#include <string>

class IntersectionClient : public ClientInterface
{
protected:
  actionlib::SimpleActionClient<custom_msgs::intersectionAction> ac_;
  custom_msgs::intersectionGoal goal_;
  ros::ServiceClient avoidingObstSetPassive_;
  ros::ServiceClient resetLaneController_;
  bool result_;

public:
  explicit IntersectionClient(std::string name);
  ~IntersectionClient();

  void setGoal(boost::any goal);
  bool waitForResult(float timeout);
  void cancelAction();
  bool waitForServer(float timeout);

  void doneCb(const actionlib::SimpleClientGoalState& state, const custom_msgs::intersectionResultConstPtr& result);
  void activeCb();
  bool getResult();
  void getActionResult(boost::any& result);
  void prepareAction();
};

#endif  // SCHEDULER_CLIENTS_INTERSECTION_ACTION_CLIENT_H
