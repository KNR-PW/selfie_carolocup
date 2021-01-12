#pragma once
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <custom_msgs/searchAction.h>
#include <custom_msgs/Motion.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <common/state_publisher.h>
#include <custom_msgs/task_enum.h>
#include <parking_spot_detector/DetectParkingSpotConfig.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/client.h>
#include <dynamic_reconfigure/server.h>

using namespace std;

enum measure_state_enum
{
    NO_BOX_DETECTED = 1,
    MEASURING_BOX,
    MEASURING_PLACE,
    FINISH_MEASURING_PLACE
};

class SearchServer
{
public:
  SearchServer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~SearchServer();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sensor_sub_;
  ros::Subscriber distance_sub_;
  ros::Publisher speed_pub_;

  actionlib::SimpleActionServer<custom_msgs::searchAction> search_server_;
  custom_msgs::searchResult result;
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
  int sensor_treshold_;
  
  int state_{selfie::TASK_SHIFTING};
  measure_state_enum measuring_state_;
  StatePublisher state_publisher_;

  float parking_spot_begin_;
  float parking_spot_end_;
  dynamic_reconfigure::Server<parking_spot_detector::DetectParkingSpotConfig> dr_server_;
  dynamic_reconfigure::Server<parking_spot_detector::DetectParkingSpotConfig>::CallbackType dr_server_CB_;
  void reconfigureCB(parking_spot_detector::DetectParkingSpotConfig& config, uint32_t level);

  void updateState(const int &state);
  bool init();
  void preemptCB();
  void endAction();
  void send_goal();
  void distanceCb(const custom_msgs::Motion&);
  void sensorCb(const std_msgs::Int32& msg);
};