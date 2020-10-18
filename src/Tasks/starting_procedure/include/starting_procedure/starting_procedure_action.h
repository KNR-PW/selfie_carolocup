/***Copyright ( c) 2020, KNR Selfie*
 * This code is licensed under BSD license (see LICENSE for details)
 **/
#ifndef STARTING_PROCEDURE_STARTING_PROCEDURE_ACTION_H
#define STARTING_PROCEDURE_STARTING_PROCEDURE_ACTION_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <string>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <starting_procedure/StartingProcedureConfig.h>
#include <dynamic_reconfigure/server.h>

#include <custom_msgs/startingAction.h>  // Note: "Action" is appended
#include <custom_msgs/Motion.h>
#include <custom_msgs/Buttons.h>
#include <custom_msgs/enums.h>
#include <custom_msgs/DriveCommand.h>

class StartingProcedureAction
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  actionlib::SimpleActionServer<custom_msgs::startingAction> as_;

  // params
  float starting_speed_;
  bool use_scan_;
  bool use_qr_;
  float Kp_;

  // create messages that are used to published feedback/result
  custom_msgs::startingGoal goal_;
  custom_msgs::startingFeedback feedback_;
  custom_msgs::startingResult result_;

  // subscribers
  ros::Subscriber button_sub_;
  ros::Subscriber distance_sub_;
  ros::Subscriber qr_sub_;
  ros::Subscriber gate_scan_sub_;
  ros::Subscriber odom_sub_;
  ros::ServiceClient qr_start_search_;
  ros::ServiceClient qr_stop_search_;
  ros::ServiceClient scan_client_;
  // publishers
  ros::Publisher drive_pub_;

  feedback_variable button_status_;

private:
  enum class State
  {
    IDLE,
    WAIT_BUTTON,
    WAIT_START,
    START_MOVE,
    END_MOVE
  };
  State state_;

  enum Buttons
  {
    PARKING = 0,
    FREE_DRIVE = 1
  };

  float distance_goal_;
  float starting_distance_;
  float distance_read_;
  tf::Pose init_pose_;
  tf::Pose current_pose_;

  ros::Time min_second_press_time_;
  ros::Duration debounce_duration_;

  void publishFeedback(feedback_variable program_state);

  void executeCB();
  void preemptCB();
  void driveBoxOut(float speed);
  void ButtonCB(const custom_msgs::Buttons& msg);
  void distanceCB(const custom_msgs::Motion& msg);
  void gateOpenCB(const std_msgs::Empty& msg);
  void odomCallback(const nav_msgs::Odometry& msg);
  dynamic_reconfigure::Server<starting_procedure::StartingProcedureConfig> dr_server_;
  dynamic_reconfigure::Server<starting_procedure::StartingProcedureConfig>::CallbackType dr_server_CB_;
  void reconfigureCB(starting_procedure::StartingProcedureConfig& config, uint32_t level);

public:
  StartingProcedureAction(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
};
#endif  // STARTING_PROCEDURE_STARTING_PROCEDURE_ACTION_H
