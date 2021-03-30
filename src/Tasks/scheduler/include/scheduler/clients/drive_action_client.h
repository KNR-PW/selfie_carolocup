/**
 *Copyright ( c ) 2019, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef SCHEDULER_CLIENTS_DRIVE_ACTION_CLIENT_H
#define SCHEDULER_CLIENTS_DRIVE_ACTION_CLIENT_H

#include <scheduler/clients/client_interface.h>

#include <custom_msgs/drivingAction.h>

#include <string>

class DriveClient : public ClientInterface
{
protected:
  actionlib::SimpleActionClient<custom_msgs::drivingAction> ac_;
  custom_msgs::drivingGoal goal_;
  bool result_;
  bool drive_mode_;

  ros::ServiceClient visionReset_;
  ros::ServiceClient resetLaneController_;
  ros::ServiceClient muxDriveSelect_;
  ros::ServiceClient avoidingObstSetPassive_;
  ros::ServiceClient avoidingObstSetActive_;

  ros::NodeHandle pnh_;

public:
  DriveClient(std::string name, const ros::NodeHandle& pnh);
  ~DriveClient();

  void setScenario(bool drive_mode);
  void setGoal(boost::any goal);
  bool waitForResult(float timeout);
  void cancelAction();
  bool waitForServer(float timeout);

  void doneCb(const actionlib::SimpleClientGoalState& state, const custom_msgs::drivingResultConstPtr& result);
  void activeCb();
  void getActionResult(boost::any& result);
  void prepareAction();
  void removeNextAction();
};

#endif  // SCHEDULER_CLIENTS_DRIVE_ACTION_CLIENT_H
