#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <vector>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/client.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/server/simple_action_server.h>

#include <custom_msgs/searchAction.h>
#include <parking_spot_detector/DetectParkingSpotConfig.h>

#include <custom_msgs/Motion.h>
#include <custom_msgs/enums.h>
#include <custom_msgs/Box2DArray.h>
#include <custom_msgs/Box2D.h>
#include <common/marker_visualization.h>

using namespace std;

class SearchServer
{
public:
  SearchServer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~SearchServer();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber obstacles_sub_;
  ros::Subscriber distance_sub_;
  ros::Publisher visualizator_pub_;
  ros::Publisher speed_pub_;

  actionlib::SimpleActionServer<custom_msgs::searchAction> search_server_;

  std::vector<custom_msgs::Box2D> boxes_on_the_right_side;  // boxes are sorted by x valule
                                                            // ascendind (near->far)
  std::vector<custom_msgs::Box2D> potential_free_places;
  custom_msgs::Box2D first_free_place;

  float length_of_parking_area_;  // length of parking area, when this distance is covered service will be aborted
  float max_distance_;
  float current_distance_;
  bool max_distance_calculated_;

  float min_spot_lenght;
  bool visualization;

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
  float ROI_min_x_;
  float ROI_max_x_;

  float ROI_min_y_;
  float ROI_max_y_;

  bool init();
  void preemptCB();
  void endAction();
  void manager(const custom_msgs::Box2DArray&);
  void distanceCb(const custom_msgs::Motion&);
  void filter_boxes(const custom_msgs::Box2DArray&);  // odfiltrowywuje boxy,
                                                      // pozostawia tylko te
                                                      // po prawej
  bool isPointInsideROI(const geometry_msgs::Point& p);
  float getDistance(geometry_msgs::Point& p1, geometry_msgs::Point& p2);
  bool find_free_places();
  void send_goal();
};
