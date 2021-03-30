/**
 *Copyright ( c ) 2019, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef SCHEDULER_SCHEDULER_H
#define SCHEDULER_SCHEDULER_H

#include <boost/any.hpp>
#include <map>
#include <std_msgs/Int8.h>

#include <scheduler/clients/client_interface.h>

#include <custom_msgs/action_enum.h>
#include <custom_msgs/task_enum.h>
#include <custom_msgs/rc_enum.h>

#include <common/state_publisher.h>

class Scheduler
{
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // params
  float start_distance_;
  float parking_spot_;
  int num_park_to_complete_;

  int park_counter_;

  ros::Subscriber task_state_sub;
  void taskStateCallback(const std_msgs::Int8ConstPtr& msg);
  selfie::EnumTask current_task_state_{ selfie::TASK_SHIFTING };
  selfie::EnumTask previous_task_state_{ selfie::TASK_SHIFTING };

  selfie::EnumAction current_action_state_{ selfie::ACTION_NONE };
  std::map<selfie::EnumAction, ClientInterface*> clients_;
  std::map<selfie::EnumAction, boost::any> action_args_;
  ClientInterface* current_client_ptr_;

  ros::Subscriber rc_state_sub;
  void rcStateCallback(const std_msgs::Int8ConstPtr& msg);
  selfie::EnumRC previous_rc_state_{ selfie::RC_UNINITIALIZED };
  selfie::EnumRC current_rc_state_{ selfie::RC_UNINITIALIZED };

  StatePublisher task_state_publisher_{ "/state/task" };

  template <typename T>
  bool checkCurrentClientType();
  void startAction(selfie::EnumAction action_to_set);
  void startNextAction();
  void stopAction();

public:
  Scheduler();
  ~Scheduler();

  void init();

  void loop();

  void actionSetup();

  void setupActionClients(bool button_pressed);
  void waitForStart();

  void actionStateMachine();
  int checkIfCurrentActionFinished();
};

#endif  // SCHEDULER_SCHEDULER_H
