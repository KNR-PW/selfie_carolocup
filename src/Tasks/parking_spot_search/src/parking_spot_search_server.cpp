#include <parking_spot_search/search_server.hpp>

SearchServer::SearchServer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh)
  , pnh_(pnh)
  , state_publisher_("/state/task")
  , search_server_(nh_, "task/parking_spot_detector", false)
  , dr_server_CB_(boost::bind(&SearchServer::reconfigureCB, this, _1, _2))
{
  search_server_.registerGoalCallback(boost::bind(&SearchServer::init, this));
  search_server_.registerPreemptCallback(boost::bind(&SearchServer::preemptCB, this));
  dr_server_.setCallback(dr_server_CB_);

  search_server_.start();
  pnh_.param<float>("default_speed_in_parking_zone", default_speed_in_parking_zone, 0.9);
  pnh_.param<float>("speed_when_found_place", speed_when_found_place, 0.3);
  pnh_.param<float>("max_distance_to_free_place", max_distance_to_free_place_, 0.8);
  pnh_.param<float>("box_angle_deg", tangens_of_box_angle_, 55);  // maximum angle between car and found place
  pnh_.param<float>("length_of_parking_area", length_of_parking_area_, 5.5);
  tangens_of_box_angle_ = tan(tangens_of_box_angle_ * M_PI / 180);

  speed_pub_ = nh_.advertise<std_msgs::Float64>("/max_speed", 5);

  speed_current.data = default_speed_in_parking_zone;
}

SearchServer::~SearchServer()
{
}

bool SearchServer::init()
{
  sensor_sub_ = nh_.subscribe("/selfie_out/sensor", 1, &SearchServer::sensorCb, this);
  distance_sub_ = nh_.subscribe("/selfie_out/motion", 1, &SearchServer::distanceCb, this);

  speed_current.data = default_speed_in_parking_zone;
  speed_pub_.publish(speed_current);
  min_spot_lenght = search_server_.acceptNewGoal()->min_spot_lenght;
  updateState(selfie::SEARCHING_PARKING_SPOT);
  ROS_INFO("Initialized");
  return true;
}

void SearchServer::distanceCb(const custom_msgs::Motion& msg)
{
  current_distance_ = msg.distance;
  if (!max_distance_calculated_)
  {
    max_distance_ = current_distance_ + length_of_parking_area_;
    max_distance_calculated_ = true;
  }
  if (max_distance_ < current_distance_)
  {
    ROS_INFO("Haven't found free place in designated distance, aborting");
    updateState(selfie::PLACE_NOT_FOUND);
    preemptCB();
  }
}

void SearchServer::sensorCb(const std_msgs::Int32& msg)
{   
  if (!search_server_.isActive())
  {
    ROS_INFO("parking_spot_search is not active");
    return;
  }
  int sensor_read = msg.data;

  switch (state_)
  {
    case selfie::SEARCHING_PARKING_SPOT:
      switch (measuring_state_) 
      {
        case NO_BOX_DETECTED:
          if (sensor_read < sensor_treshold_) 
          {
            measuring_state_ = MEASURING_BOX;
          }
          break;
        case MEASURING_BOX:
          if (sensor_read > sensor_treshold_) 
          {
            measuring_state_ = MEASURING_PLACE;
            parking_spot_begin_ = current_distance_;
          }
          break;
        case MEASURING_PLACE:
          if (sensor_read < sensor_treshold_)
          {
            parking_spot_end_ = current_distance_;
            if(parking_spot_end_ - parking_spot_begin_ >  min_spot_lenght)
            {
              updateState(selfie::PLACE_PROPER_FOUND);
            }
            else
            {
              measuring_state_ = MEASURING_BOX;
            }
          }
          break;
      }
      break;
    case selfie::PLACE_PROPER_FOUND:
      send_goal();
      break;
  }
}

void SearchServer::reconfigureCB(parking_spot_detector::DetectParkingSpotConfig& config, uint32_t level)
{

}

void SearchServer::updateState(const int &state)
{
  state_publisher_.updateState(state);
  state_ = state;
}

void SearchServer::preemptCB()
{
  ROS_INFO("Action preempted");
  endAction();
  search_server_.setAborted();
}

void SearchServer::endAction()  // shutting donw unnecesary subscribers and publishers
{
  distance_sub_.shutdown();
  sensor_sub_.shutdown();
  max_distance_calculated_ = false;
}

void SearchServer::send_goal()
{
  result.parking_spot = first_free_place;
  ROS_INFO("Place found and sent");
  search_server_.setSucceeded(result);
  endAction();
}