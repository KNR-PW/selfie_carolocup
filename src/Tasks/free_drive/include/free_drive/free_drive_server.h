/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/ 

#ifndef FREE_DRIVE_FREE_DRIVE_SERVER_H
#define FREE_DRIVE_FREE_DRIVE_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <string>
#include <chrono>  // NOLINT [build/c++11]
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <dynamic_reconfigure/server.h>

#include <custom_msgs/drivingAction.h>  // Note: "Action" is appended
#include <custom_msgs/enums.h>
#include <free_drive/FreeDriveConfig.h>

class FreeDriveServer
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  actionlib::SimpleActionServer<custom_msgs::drivingAction> as_;

  // create messages that are used to published feedback/result
  custom_msgs::drivingFeedback feedback_;
  custom_msgs::drivingResult result_;
  custom_msgs::drivingGoal goal_;

  // subscribers
  ros::Subscriber starting_line_sub_;
  ros::Subscriber intersection_sub_;
  ros::Subscriber distance_sub_;

  // publishers
  ros::Publisher max_speed_pub_;

  float distance_to_event_;
  bool event_detected_;
  float event_distance_to_end_;
  std::chrono::steady_clock::time_point last_event_time_;

  float starting_line_distance_to_end_;
  float intersection_distance_to_end_;

  bool event_verified_                 {true};
  float distance_on_last_event_        {0.0};
  float distance_to_verify_event_      {2.0};

  float max_speed_;
  int last_feedback_ {AUTONOMOUS_DRIVE};

  dynamic_reconfigure::Server<free_drive::FreeDriveConfig> dr_server_;
  dynamic_reconfigure::Server<free_drive::FreeDriveConfig>::CallbackType dr_server_CB_;
  void reconfigureCB(free_drive::FreeDriveConfig& config, uint32_t level);


public:
  FreeDriveServer(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
  ~FreeDriveServer(void);

  inline void publishFeedback(feedback_variable program_state);
  inline void maxSpeedPub();

  void registerGoal();
  void executeLoop();
  void preemptCB();
  void startingLineCB(const std_msgs::Float32 &msg);
  void intersectionCB(const std_msgs::Float32 &msg);
  void distanceCB(const std_msgs::Float32 &msg);
};
#endif  // FREE_DRIVE_FREE_DRIVE_SERVER_H
