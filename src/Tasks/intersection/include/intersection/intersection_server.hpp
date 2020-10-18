#pragma once

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include <custom_msgs/Box2DArray.h>
#include <custom_msgs/Box2D.h>
#include <custom_msgs/Motion.h>
#include <custom_msgs/IntersectionStop.h>
#include <custom_msgs/intersectionAction.h>
#include <std_msgs/Float64.h>

#include <dynamic_reconfigure/server.h>
#include <intersection/IntersectionServerConfig.h>
#include <custom_msgs/enums.h>
#include <common/marker_visualization.h>

class IntersectionServer
{
public:
  IntersectionServer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~IntersectionServer()
  {
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber obstacles_sub_;
  ros::Subscriber intersection_subscriber_;
  ros::Subscriber distance_subscriber_;
  ros::Publisher speed_publisher_;
  ros::Publisher visualize_intersection_;

  double beginning_time_;
  double current_time_;
  double difftime_;
  bool time_started_;
  float stop_time_;  // How long car should stop when approached intersection
  float speed_default_;

  float point_min_x_;  // Area of interest
  float point_max_x_;
  float point_min_y_;
  float point_max_y_;
  float road_width_;
  float max_distance_to_intersection_;  // Describrs how far before intersection
                                        // car should stop
  float distance_of_blind_approaching_;
  float distance_to_stop_blind_approaching_;
  float current_distance_;
  bool approached_blindly_;
  float distance_to_intersection_when_started_;
  bool is_distance_to_intersection_saved_;
  float distance_when_started_;
  bool is_distance_saved_;

  int num_corners_to_detect_;
  bool visualization_;

  std::vector<custom_msgs::Box2D> filtered_boxes_;
  custom_msgs::intersectionFeedback action_status_;
  custom_msgs::intersectionGoal goal_;
  std_msgs::Float64 speed_;

  actionlib::SimpleActionServer<custom_msgs::intersectionAction> intersectionServer_;

  dynamic_reconfigure::Server<intersection::IntersectionServerConfig> dr_server_;
  dynamic_reconfigure::Server<intersection::IntersectionServerConfig>::CallbackType dr_server_CB_;

  void init();
  void preemptCb();
  void manager(const custom_msgs::Box2DArray&);
  void intersectionCallback(const custom_msgs::IntersectionStop&);
  void distanceCallback(const custom_msgs::Motion&);
  void filterBoxes(const custom_msgs::Box2DArray&);
  bool isPointInsideROI(const geometry_msgs::Point& p);
  void publishFeedback(program_state newStatus);
  void sendGoal();
  void reconfigureCB(intersection::IntersectionServerConfig& config, uint32_t level);
};