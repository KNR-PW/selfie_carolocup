/**
 *Copyright ( c ) 2019, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/
#ifndef LANE_PILOT_ROAD_OBSTACLE_DETECTOR_H
#define LANE_PILOT_ROAD_OBSTACLE_DETECTOR_H

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <list>
#include <ros/ros.h>
#include <ros/console.h>

#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/client.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#include <lane_pilot/LaneControllerConfig.h>
#include <common/marker_visualization.h>
#include <common/state_publisher.h>
#include <custom_msgs/Indicators.h>
#include <custom_msgs/Motion.h>
#include <custom_msgs/Box2DArray.h>
#include <custom_msgs/Box2D.h>
#include <custom_msgs/RoadLines.h>
#include <custom_msgs/lane_control_enum.h>

using dynamic_reconfigure::Client;
using selfie::EnumLaneControl;

class RoadObstacleDetector
{
public:
  RoadObstacleDetector(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~RoadObstacleDetector();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber obstacles_sub_;
  ros::Subscriber road_lines_sub_;
  ros::Subscriber motion_sub_;
<<<<<<< HEAD
=======
  ros::Subscriber overtaking_sub_;
  ros::Subscriber speed_limit_sub_;
>>>>>>> e39564f... Implement speed limiter
  ros::Publisher speed_pub_;
  ros::Publisher visualizer_;
  ros::Publisher offset_pub_;
  ros::Publisher indicators_pub_;
  // Two services as switches activating active/passive mode
  ros::ServiceServer passive_mode_service_;
  ros::ServiceServer active_mode_service_;
  ros::ServiceServer reset_node_service_;
  ros::Timer timer_;

  // Polymonial coefficients describing road markings
  float left_line_[4];
  float center_line_[4];
  float right_line_[4];

  // area of interest (camera's field of view)
  float ROI_min_x_;
  float ROI_max_x_;
  float ROI_min_y_;
  float ROI_max_y_;

  // area of interest for checking if there is any car on right (while overtaking)
  float right_obst_area_min_x_;
  float right_obst_area_max_x_;
  float right_obst_area_min_y_;
  float right_obst_area_max_y_;
  // Offsets for lanes
  float right_lane_offset_;
  float left_lane_offset_;

  float max_speed_;
  float slowdown_speed_;
  float lane_change_speed_;
  std_msgs::Float64 speed_message_;

  float max_distance_to_obstacle_;  // to avoid changing lane too early
  float max_length_of_obstacle_;

  float safety_margin_;  // safety margin considering inaccurations in measuring distance etc..
  float current_distance_;
  float current_offset_;
  float return_distance_;  // after passing this distance car returns on right lane

  float lane_change_distance_;                 // returning from left lane should be gradual, and it should take about
                                               // "lane_change_distance_" meters
  float distance_when_started_changing_lane_;  // saved when we begin changing lane

<<<<<<< HEAD
=======
  float target_distance_to_obstacle_;
  bool can_overtake_;
  bool speed_limit_;
>>>>>>> e39564f... Implement speed limiter
  int proof_slowdown_;
  int num_proof_to_slowdown_;
  int proof_return_;
  int num_proof_to_return_;
  int num_corners_to_detect_;

  StatePublisher state_publisher_;

  std::list<custom_msgs::Box2D> filtered_boxes_;  // boxes are sorted by x value
                                                  // ascendend (near->far)
  std::list<custom_msgs::Box2D>::iterator nearest_box_in_front_of_car_;
  std_msgs::Float64 offset_value_;

  bool visualization_;
  bool are_road_lines_received_;
  bool return_distance_calculated_;
  selfie::EnumLaneControl state_;

  custom_msgs::Box2D area_of_interest_box_;
  custom_msgs::Box2D right_obst_area_box_;  // scanned area on the right line when we are on the left one

  dynamic_reconfigure::Server<lane_pilot::LaneControllerConfig> dr_server_;
  dynamic_reconfigure::Server<lane_pilot::LaneControllerConfig>::CallbackType dr_server_CB_;

  void reconfigureCB(const lane_pilot::LaneControllerConfig& config, uint32_t level);
  void updateState(const selfie::EnumLaneControl& state);

  void filterBoxes(const custom_msgs::Box2DArray&);       // filters boxes and saves in filtered_boxes_
  void roadLinesCallback(const custom_msgs::RoadLines&);  // checks if boxes from filtered_boxes_ are on right
                                                          // lane
  void obstacleCallback(const custom_msgs::Box2DArray&);
  void motionCallback(const custom_msgs::Motion&);
<<<<<<< HEAD
=======
  void overtakingCallback(const std_msgs::Bool&);
  void speedCallback(const std_msgs::Bool&);
>>>>>>> e39564f... Implement speed limiter
  void calculateReturnDistance();

  bool switchToActive(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  bool switchToPassive(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  bool resetNode(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  void sendIndicators(bool is_left_on, bool is_right_on);

  bool isPointOnRightLane(const geometry_msgs::Point&);
  bool isObstacleNextToCar(const custom_msgs::Box2DArray&);
  void passiveTimerCallback(const ros::TimerEvent&);

  void visualizeBoxes();

  bool isPointInsideROI(const geometry_msgs::Point& p,
                        const float& ROI_min_x,
                        const float& ROI_max_x,
                        const float& ROI_min_y,
                        const float& ROI_max_y);
};

#endif  // LANE_PILOT_ROAD_OBSTACLE_DETECTOR_H
