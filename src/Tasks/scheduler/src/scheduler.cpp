/**
 *Copyright ( c ) 2019, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#include <scheduler/scheduler.h>
#include <scheduler/clients/starting_action_client.h>
#include <scheduler/clients/drive_action_client.h>
#include <scheduler/clients/search_action_client.h>
#include <scheduler/clients/park_action_client.h>
#include <scheduler/clients/intersection_action_client.h>

Scheduler::Scheduler() : pnh_("~"), start_distance_(1.0), parking_spot_(0.6), num_park_to_complete_(2), park_counter_(0)
{
  pnh_.getParam("starting_distance", start_distance_);
  pnh_.getParam("parking_spot", parking_spot_);
  pnh_.getParam("num_park_to_complete", num_park_to_complete_);

  ROS_INFO(
      "Created scheduler with params: PC: %d SD: %f, PS: %f", num_park_to_complete_, start_distance_, parking_spot_);

  task_state_sub = nh_.subscribe("/state/task", 10, &Scheduler::taskStateCallback, this);
  rc_state_sub = nh_.subscribe("/state/rc", 10, &Scheduler::rcStateCallback, this);

  clients_[selfie::STARTING_PROCEDURE] = new StartingProcedureClient("task/starting_procedure");
  action_args_[selfie::STARTING_PROCEDURE] = start_distance_;

  clients_[selfie::FREE_DRIVE] = new DriveClient("task/free_drive", pnh_);
  action_args_[selfie::FREE_DRIVE] = [](bool x) { return x; }(false);

  ROS_INFO("Clients created successfully");
}

Scheduler::~Scheduler()
{
  pnh_.deleteParam("begin_action");
  pnh_.deleteParam("starting_distance");
  pnh_.deleteParam("parking_spot");
}

void Scheduler::waitForStart()
{
  while (ros::ok())
  {
    if (checkIfCurrentActionFinished() == SUCCESS)
    {
      boost::any result;
      current_client_ptr_->getActionResult(result);

      DriveClient* drive_client = dynamic_cast<DriveClient*>(clients_[selfie::FREE_DRIVE]);
      bool drive_mode = boost::any_cast<bool>(result);

      drive_client->setScenario(drive_mode);
      setupActionClients(drive_mode);
      break;
    }
    else if (checkIfCurrentActionFinished() == ABORTED)
      break;
  }
}

void Scheduler::setupActionClients(bool button_pressed)
{
  if (button_pressed == 0)  // parking mode
  {
    clients_[selfie::PARKING_SPOT_SEARCH] = new SearchClient("task/parking_spot_detector");
    action_args_[selfie::PARKING_SPOT_SEARCH] = parking_spot_;

    clients_[selfie::PARK] = new ParkClient("task/park", pnh_);
    action_args_[selfie::PARK] = [](custom_msgs::Box2D x) { return x; };
  }
  else  // intersection mode
  {
    clients_[selfie::INTERSECTION_STOP] = new IntersectionClient("task/intersection");
    action_args_[selfie::INTERSECTION_STOP] = static_cast<int>(0);  // empty goal
  }
}

void Scheduler::init()
{
  startAction(selfie::STARTING_PROCEDURE);
}

void Scheduler::startAction(selfie::EnumAction action_to_set)
{
  task_state_publisher_.updateState(selfie::TASK_SHIFTING);
  current_action_state_ = action_to_set;
  current_client_ptr_ = clients_[action_to_set];
  current_client_ptr_->waitForServer(200);
  current_client_ptr_->prepareAction();
  current_client_ptr_->setGoal(action_args_[action_to_set]);
}

void Scheduler::startNextAction()
{
  selfie::EnumAction next_action = current_client_ptr_->getNextAction();

  if (next_action == current_action_state_)
  {
    return;
  }

  task_state_publisher_.updateState(selfie::TASK_SHIFTING);
  current_client_ptr_ = clients_[next_action];
  current_action_state_ = next_action;
  current_client_ptr_->waitForServer(200);
  current_client_ptr_->prepareAction();
  current_client_ptr_->setGoal(action_args_[next_action]);
}

int Scheduler::checkIfCurrentActionFinished()
{
  return current_client_ptr_->getGoalState();
}

void Scheduler::loop()
{
  if (checkIfCurrentActionFinished() == SUCCESS)
  {
    if (current_action_state_ == selfie::PARK)
    {
      ++park_counter_;
      if (park_counter_ >= num_park_to_complete_)
      {
        DriveClient* drive_client = dynamic_cast<DriveClient*>(clients_[selfie::FREE_DRIVE]);
        drive_client->removeNextAction();
      }
    }
    current_client_ptr_->getActionResult(action_args_[current_client_ptr_->getNextAction()]);
    startNextAction();
  }
  else if (checkIfCurrentActionFinished() == ABORTED)
  {
    // abort caused by RC
    if (current_rc_state_ == selfie::RC_MANUAL)
    {
      // empty state
    }
    else  // abort caused by server
    {
      stopAction();
      startAction(selfie::FREE_DRIVE);
    }
  }
}

template <typename T>
bool Scheduler::checkCurrentClientType()
{
  ClientInterface* check = dynamic_cast<T>(current_client_ptr_);
  if (check)
  {
    return true;
  }
  return false;
}

void Scheduler::stopAction()
{
  current_client_ptr_->cancelAction();
  ROS_WARN("STOP current action");
}

void Scheduler::taskStateCallback(const std_msgs::Int8ConstPtr& msg)
{
  previous_task_state_ = current_task_state_;
  current_task_state_ = static_cast<selfie::EnumTask>(msg->data);
}

void Scheduler::rcStateCallback(const std_msgs::Int8ConstPtr& msg)
{
  previous_rc_state_ = current_rc_state_;
  current_rc_state_ = static_cast<selfie::EnumRC>(msg->data);

  // prevent from execution on the beginning
  if (current_task_state_ != selfie::TASK_SHIFTING && current_task_state_ != selfie::WAITING_FOR_BUTTON)
  {
    if (current_rc_state_ == selfie::RC_MANUAL &&
        (previous_rc_state_ == selfie::RC_AUTONOMOUS || previous_rc_state_ == selfie::RC_HALF_AUTONOMOUS))
    {
      stopAction();
    }
    else if ((current_rc_state_ == selfie::RC_AUTONOMOUS || current_rc_state_ == selfie::RC_HALF_AUTONOMOUS) &&
             previous_rc_state_ == selfie::RC_MANUAL)
    {
      startAction(selfie::FREE_DRIVE);
    }
  }
}
