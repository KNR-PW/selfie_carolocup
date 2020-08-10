#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <vector>

#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/client.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <custom_msgs/PolygonArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <visualization_msgs/Marker.h>

#include <actionlib/server/simple_action_server.h>
#include <custom_msgs/searchAction.h>
#include <parking_spot_detector/DetectParkingSpotConfig.h>
#include <custom_msgs/enums.h>

#include <ros/console.h>

#include <common/obstacle_box.h>

using namespace std;

class Search_server
{
public:
  Search_server(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~Search_server();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber obstacles_sub;
  ros::Subscriber distance_sub_;
  ros::Publisher visualize_free_place;
  ros::Publisher speed_publisher;

  actionlib::SimpleActionServer<custom_msgs::searchAction> search_server_;

  std::vector<Box> boxes_on_the_right_side;  // boxes are sorted by x valule
                                             // ascendind (near->far)
  std::vector<Box> potential_free_places;
  Box first_free_place;

  float length_of_parking_area_;  // length of parking area, when this distance is covered service will be aborted
  float max_distance_;
  float current_distance_;
  bool max_distance_calculated_;

  float min_spot_lenght;
  bool visualization;
  Box area_of_interest_;

  float tangens_of_box_angle_;  // describes max deviation
  float max_distance_to_free_place_;
  float default_speed_in_parking_zone;
  float speed_when_found_place;
  std_msgs::Float64 speed_current;

  custom_msgs::searchFeedback action_status;
  void publishFeedback(unsigned int);
  custom_msgs::searchResult result;

  dynamic_reconfigure::Server<parking_spot_detector::DetectParkingSpotConfig> dr_server_;
  dynamic_reconfigure::Server<parking_spot_detector::DetectParkingSpotConfig>::CallbackType dr_server_CB_;
  void reconfigureCB(parking_spot_detector::DetectParkingSpotConfig& config, uint32_t level);

  // area of interest (used unit- meter)
  float point_min_x;
  float point_max_x;

  float point_min_y;
  float point_max_y;

  bool init();
  void preemptCB();
  void endAction();
  void manager(const custom_msgs::PolygonArray&);
  void distanceCb(const std_msgs::Float32&);
  void filter_boxes(const custom_msgs::PolygonArray&);  // odfiltrowywuje boxy,
                                                        // pozostawia tylko te
                                                        // po prawej
  bool find_free_places();
  void send_goal();

  void display_places(std::vector<Box>&, const std::string&);
  void display_place(Box&, const std::string&, float = 100.0f, float = 255.0f, float = 200.0f);
};
