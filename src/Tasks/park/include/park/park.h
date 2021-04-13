/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef PARK_PARK_H
#define PARK_PARK_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <custom_msgs/parkAction.h>
#include <custom_msgs/Motion.h>
#include <custom_msgs/Indicators.h>
#include <custom_msgs/DriveCommand.h>
#include <vector>
#include <string>
#include <park/ParkConfig.h>
#include <dynamic_reconfigure/server.h>
#include <algorithm>
#include <std_srvs/Empty.h>
#include <custom_msgs/RoadLines.h>
#include <custom_msgs/Box2D.h>

#include <common/state_publisher.h>
#include <custom_msgs/task_enum.h>

class Park
{
public:
  Park(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

private:
  const float PARK_SPOT_WIDTH = 0.3;

  ros::Subscriber dist_sub_;
  ros::NodeHandle nh_, pnh_;
  actionlib::SimpleActionServer<custom_msgs::parkAction> as_;
  ros::Publisher drive_pub_;
  ros::Publisher indicator_pub_;
  ros::Subscriber markings_sub_;
  ros::ServiceClient steering_mode_set_parallel_;
  ros::ServiceClient steering_mode_set_front_axis_;

  dynamic_reconfigure::Server<park::ParkConfig> dr_server_;
  dynamic_reconfigure::Server<park::ParkConfig>::CallbackType dr_server_CB_;
  void reconfigureCB(const park::ParkConfig& config, uint32_t level);

  void distanceCallback(const custom_msgs::Motion& msg);
  void markingsCallback(const custom_msgs::RoadLines& msg);
  void goalCB();
  void preemptCB();
  void updateState(const int& state);

  void drive(float speed, float steering_angle_front, float steering_angle_rear);
  bool toParkingSpot();
  bool park();
  bool leave();
  void initParkingSpot(const custom_msgs::Box2D& msg);
  void blink(bool left, bool right);

  enum Parking_State
  {
    not_parking = 0,
    go_to_parking_spot = 1,
    going_in = 2,
    parked = 3,
    get_straight = 7,
    going_out = 4,
    out = 5,
    go_back = 6
  };
  Parking_State parking_state_;

  int state_{ selfie::TASK_SHIFTING };
  StatePublisher state_publisher_;

  float parking_speed_;

  float actual_dist_;
  float prev_dist_;

  float park_spot_dist_;
  float park_spot_dist_ini_;

  float front_target_;
  float back_target_;
  float park_spot_middle_;
  std::vector<float> right_line_;
  ros::Time delay_end_;
  float out_target_;

  enum Move_State
  {
    first_phase = 0,
    straight = 1,
    second_phase = 2,
    end = 3
  };
  Move_State move_state_;

  // params
  std::string ackermann_topic_;
  float minimal_start_parking_x_;
  bool state_msgs_;
  float max_distance_to_wall_;
  float max_turn_;
  float idle_time_;
  float iter_distance_;
  float back_to_mid_;
  std::string odom_topic_;
  float angle_coeff_;
  float turn_delay_;
  float line_dist_end_;
  float start_parking_speed_;
};
#endif  // PARK_PARK_H
