/**
 *Copyright ( c ) 2019, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#include <lane_pilot/lane_pilot.hpp>

RoadObstacleDetector::RoadObstacleDetector(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh)
  , pnh_(pnh)
  , state_publisher_("/state/lane_control")
  , are_road_lines_received_(false)
  , max_distance_to_obstacle_(0.5)
  , proof_slowdown_(0)
  , num_corners_to_detect_(3)
  , current_distance_(0)
  , current_offset_(0)
  , return_distance_calculated_(false)
  , state_(EnumLaneControl::UNINITIALIZED)
  , dr_server_CB_(boost::bind(&RoadObstacleDetector::reconfigureCB, this, _1, _2))
{
  pnh_.param<bool>("visualization", visualization_, true);
  pnh_.param<float>("max_length_of_obstacle", max_length_of_obstacle_, 0.8);
  pnh_.param<float>("max_distance_to_obstacle", max_distance_to_obstacle_, 0.5);
  pnh_.param<float>("ROI_min_x", ROI_min_x_, 0.3);
  pnh_.param<float>("ROI_max_x", ROI_max_x_, 1.1);
  pnh_.param<float>("ROI_min_y", ROI_min_y_, -1.3);
  pnh_.param<float>("ROI_max_y", ROI_max_y_, 1.3);
  pnh_.param<float>("right_obst_area_min_x", right_obst_area_min_x_, -0.5);
  pnh_.param<float>("right_obst_area_max_x", right_obst_area_max_x_, 1);
  pnh_.param<float>("right_obst_area_min_y", right_obst_area_min_y_, 0.1);
  pnh_.param<float>("right_obst_area_max_y", right_obst_area_max_y_, 1.3);
  pnh_.param<float>("right_lane_offset", right_lane_offset_, -0.2);
  pnh_.param<float>("left_lane_offset", left_lane_offset_, 0.2);
  pnh_.param<float>("maximum_speed", max_speed_, 0.3);
  pnh_.param<float>("slowdown_speed", slowdown_speed_, 0.1);
  pnh_.param<float>("lane_change_speed", lane_change_speed_, 0.1);
  pnh_.param<float>("safety_margin", safety_margin_, 1.15);
  pnh_.param<int>("num_proof_to_slowdown", num_proof_to_slowdown_, 2);
  pnh_.param<int>("num_corners_to_detect", num_corners_to_detect_, 3);
  pnh_.param<float>("lane_change_distance", lane_change_distance_, 0.9);
  pnh_.param<float>("target_distance_to_obstacle", target_distance_to_obstacle_, 0.5);

  num_proof_to_return_ = num_proof_to_slowdown_;  // Maybe change to param later
  dr_server_.setCallback(dr_server_CB_);
  passive_mode_service_ =
      nh_.advertiseService("/avoiding_obst_set_passive", &RoadObstacleDetector::switchToPassive, this);
  active_mode_service_ = nh_.advertiseService("/avoiding_obst_set_active", &RoadObstacleDetector::switchToActive, this);
  reset_node_service_ = nh_.advertiseService("/resetLaneControl", &RoadObstacleDetector::resetNode, this);
  offset_pub_ = nh_.advertise<std_msgs::Float64>("/path_offset", 1);
  speed_pub_ = nh_.advertise<std_msgs::Float64>("/max_speed", 1);
  indicators_pub_ = nh_.advertise<custom_msgs::Indicators>("/selfie_in/indicators", 20, true);

  speed_message_.data = max_speed_;

  if (visualization_)
  {
    visualizer_ = nh_.advertise<visualization_msgs::Marker>("/visualization/avoiding_obstacles", 1);

    geometry_msgs::Point p;

    // Initializing Box describing area of interest
    p.x = ROI_min_x_;
    p.y = ROI_min_y_;
    area_of_interest_box_.br = p;
    p.x = ROI_min_x_;
    p.y = ROI_max_y_;
    area_of_interest_box_.bl = p;
    p.x = ROI_max_x_;
    p.y = ROI_min_y_;
    area_of_interest_box_.tr = p;
    p.x = ROI_max_x_;
    p.y = ROI_max_y_;
    area_of_interest_box_.tl = p;

    // Initializing Box describing area of searching right box
    p.x = right_obst_area_min_x_;
    p.y = right_obst_area_min_y_;
    right_obst_area_box_.br = p;
    p.x = right_obst_area_min_x_;
    p.y = right_obst_area_max_y_;
    right_obst_area_box_.bl = p;
    p.x = right_obst_area_max_x_;
    p.y = right_obst_area_min_y_;
    right_obst_area_box_.tr = p;
    p.x = right_obst_area_max_x_;
    p.y = right_obst_area_max_y_;
    right_obst_area_box_.tl = p;
  }
  offset_value_.data = right_lane_offset_;
  timer_ = nh_.createTimer(ros::Duration(0.5), &RoadObstacleDetector::passiveTimerCallback, this);
  timer_.start();

  ROS_INFO("road_obstacle_detector initialized ");
  updateState(EnumLaneControl::PASSIVE_RIGHT);
}

RoadObstacleDetector::~RoadObstacleDetector()
{
}

void RoadObstacleDetector::updateState(const selfie::EnumLaneControl& state)
{
  state_publisher_.updateState(state);
  state_ = state;
}

void RoadObstacleDetector::overtakingCallback(const std_msgs::Bool& msg)
{
  can_overtake_ = msg.data;
}

void RoadObstacleDetector::obstacleCallback(const custom_msgs::Box2DArray& msg)
{
  if (state_ != EnumLaneControl::OVERTAKE && state_ != EnumLaneControl::ON_LEFT)
  {
    filterBoxes(msg);
    if (!filtered_boxes_.empty())
    {
      ++proof_slowdown_;
      if (!can_overtake_)
      {
        float distance = std::min(nearest_box_in_front_of_car_->bl.x,
                                  nearest_box_in_front_of_car_->br.x);

        if (distance > 2 * target_distance_to_obstacle_)
        {
          speed_message_.data = max_speed_;
          offset_value_.data = right_lane_offset_;
          speed_pub_.publish(speed_message_);
          offset_pub_.publish(offset_value_);
          return;
        }

        float error = target_distance_to_obstacle_ - distance;
        if (error < 0)
        {
          speed_message_.data = std::min(std::abs(max_speed_ * error), max_speed_);
        }
        else
        {
          speed_message_.data = 0;
        }
        offset_value_.data = right_lane_offset_;

        speed_pub_.publish(speed_message_);
        offset_pub_.publish(offset_value_);
        return;
      }
      else if ((state_ == EnumLaneControl::ON_RIGHT || state_ == EnumLaneControl::RETURN_RIGHT) &&
              (nearest_box_in_front_of_car_->bl.x <= max_distance_to_obstacle_ ||
               nearest_box_in_front_of_car_->br.x <= max_distance_to_obstacle_))
      {
        proof_slowdown_ = 0;
        calculateReturnDistance();
        distance_when_started_changing_lane_ = current_distance_;
        ROS_INFO("LC: OVERTAKE");
        updateState(EnumLaneControl::OVERTAKE);
      }
    }
    else
    {
      if (proof_slowdown_ > 0)
      {
        --proof_slowdown_;
      }
    }
  }
  else if (state_ == EnumLaneControl::ON_LEFT)
  {
    if (!isObstacleNextToCar(msg))
    {
      proof_return_++;
    }
    else if (proof_return_ > 0)
    {
      proof_return_--;
    }

    if (proof_return_ > num_proof_to_return_)
    {
      updateState(EnumLaneControl::RETURN_RIGHT);
      ROS_INFO("LC: RETURN");
      return_distance_calculated_ = false;
      distance_when_started_changing_lane_ = current_distance_;
      custom_msgs::Box2DArray temp;
      obstacleCallback(temp);
    }
  }
  switch (state_)
  {
    case EnumLaneControl::ON_RIGHT:
      offset_value_.data = right_lane_offset_;

      if (proof_slowdown_ >= num_proof_to_slowdown_)
        speed_message_.data = slowdown_speed_;
      else
        speed_message_.data = max_speed_;

      break;
    case EnumLaneControl::OVERTAKE:
      sendIndicators(true, false);
      offset_value_.data = left_lane_offset_;
      speed_message_.data = lane_change_speed_;
      break;
    case EnumLaneControl::ON_LEFT:
      offset_value_.data = left_lane_offset_;
      speed_message_.data = max_speed_;
      break;
    case EnumLaneControl::RETURN_RIGHT:
      sendIndicators(false, true);
      offset_value_.data = right_lane_offset_;
      speed_message_.data = lane_change_speed_;
      break;
    case EnumLaneControl::UNINITIALIZED:
      return;
    default:
      ROS_ERROR("Wrong avoiding_obstacle action status");
  }
  offset_pub_.publish(offset_value_);
  speed_pub_.publish(speed_message_);
}

void RoadObstacleDetector::filterBoxes(const custom_msgs::Box2DArray& msg)
{
  filtered_boxes_.clear();

  for (const custom_msgs::Box2D& box : msg.boxes)
  {
    int corners_ok = 0;
    if (isPointOnRightLane(box.bl) && isPointInsideROI(box.bl, ROI_min_x_, ROI_max_x_, ROI_min_y_, ROI_max_y_))
    {
      ++corners_ok;
    }
    if (isPointOnRightLane(box.br) && isPointInsideROI(box.br, ROI_min_x_, ROI_max_x_, ROI_min_y_, ROI_max_y_))
    {
      ++corners_ok;
    }
    if (isPointOnRightLane(box.tl) && isPointInsideROI(box.tl, ROI_min_x_, ROI_max_x_, ROI_min_y_, ROI_max_y_))
    {
      ++corners_ok;
    }
    if (isPointOnRightLane(box.tr) && isPointInsideROI(box.tr, ROI_min_x_, ROI_max_x_, ROI_min_y_, ROI_max_y_))
    {
      ++corners_ok;
    }

    if (corners_ok >= num_corners_to_detect_)
    {
      filtered_boxes_.insert(filtered_boxes_.begin(), box);
      if (box.bl.x > 0)
      {
        nearest_box_in_front_of_car_ = filtered_boxes_.begin();
      }
    }
  }

  if (visualization_)
  {
    visualizeBoxes();
  }
}

bool RoadObstacleDetector::isPointInsideROI(const geometry_msgs::Point& p,
                                            const float& ROI_min_x,
                                            const float& ROI_max_x,
                                            const float& ROI_min_y,
                                            const float& ROI_max_y)
{
  return !(p.x < ROI_min_x || p.x > ROI_max_x || p.y < ROI_min_y || p.y > ROI_max_y);
}

bool RoadObstacleDetector::isObstacleNextToCar(const custom_msgs::Box2DArray& msg)
{
  if (visualization_)
    selfie::visualizeBox2D(right_obst_area_box_, visualizer_, "area_of_right_boxes", 1, 0.9, 0.7);

  for (const custom_msgs::Box2D& box : msg.boxes)
  {
    int corners_ok = 0;
    if (isPointInsideROI(
            box.br, right_obst_area_min_x_, right_obst_area_max_x_, right_obst_area_min_y_, right_obst_area_max_y_))
    {
      ++corners_ok;
    }

    if (isPointInsideROI(
            box.bl, right_obst_area_min_x_, right_obst_area_max_x_, right_obst_area_min_y_, right_obst_area_max_y_))
    {
      ++corners_ok;
    }

    if (isPointInsideROI(
            box.tr, right_obst_area_min_x_, right_obst_area_max_x_, right_obst_area_min_y_, right_obst_area_max_y_))
    {
      ++corners_ok;
    }

    if (isPointInsideROI(
            box.tl, right_obst_area_min_x_, right_obst_area_max_x_, right_obst_area_min_y_, right_obst_area_max_y_))
    {
      ++corners_ok;
    }

    if (corners_ok >= num_corners_to_detect_)
    {
      if (visualization_)
      {
        selfie::visualizeBox2D(box, visualizer_, "box_on_right", 0.9, 0.2, 0.3);
      }
      return true;
    }
  }
  return false;
}

void RoadObstacleDetector::roadLinesCallback(const custom_msgs::RoadLines& msg)
{
  int size = msg.left_line.size();
  int i = 0;
  for (; i < size; i++)
  {
    left_line_[i] = msg.left_line[i];
    center_line_[i] = msg.center_line[i];
    right_line_[i] = msg.right_line[i];
  }
  for (; i < 4; i++)
  {
    left_line_[i] = 0;
    center_line_[i] = 0;
    right_line_[i] = 0;
  }
  are_road_lines_received_ = true;
}

bool RoadObstacleDetector::isPointOnRightLane(const geometry_msgs::Point& point)
{
  if (are_road_lines_received_ == false)
    return false;

  float right_value = right_line_[0] + point.x * right_line_[1] + point.x * point.x * right_line_[2] +
                      point.x * point.x * point.x * right_line_[3];
  float center_value = center_line_[0] + point.x * center_line_[1] + point.x * point.x * center_line_[2] +
                       point.x * point.x * point.x * center_line_[3];

  if (point.y > right_value && point.y < center_value)
    return true;
  else
    return false;
}

void RoadObstacleDetector::calculateReturnDistance()
{
  return_distance_calculated_ = true;
  return_distance_ = safety_margin_ * (max_length_of_obstacle_) + current_distance_ + lane_change_distance_;
  ROS_INFO("LC: return_distance_: %f", return_distance_);
}

void RoadObstacleDetector::visualizeBoxes()
{
  selfie::visualizeBoxes2D(filtered_boxes_, visualizer_, "boxes_on_lane", 0.9, 0.9, 0.9);
  selfie::visualizeBox2D(area_of_interest_box_, visualizer_, "area_of_interest", 1, 1, 1);
  if (filtered_boxes_.empty())
  {
    selfie::visualizeEmpty(visualizer_, "nearest_box", 1, 0.1, 0.1);
  }
  else
    selfie::visualizeBox2D(*nearest_box_in_front_of_car_, visualizer_, "nearest_box", 1, 0.1, 0.1);
}

void RoadObstacleDetector::motionCallback(const custom_msgs::Motion& msg)
{
  current_distance_ = msg.distance;
  if (state_ == EnumLaneControl::OVERTAKE &&
      current_distance_ - distance_when_started_changing_lane_ > lane_change_distance_)
  {
    updateState(EnumLaneControl::ON_LEFT);
    proof_return_ = 0;
    ROS_INFO("LC: ON_LEFT");
    sendIndicators(false, false);
    custom_msgs::Box2DArray temp;
    obstacleCallback(temp);
  }
  else if (state_ == EnumLaneControl::RETURN_RIGHT &&
           current_distance_ - distance_when_started_changing_lane_ > lane_change_distance_)
  {
    updateState(EnumLaneControl::ON_RIGHT);
    ROS_INFO("LC: ON_RIGHT");
    sendIndicators(false, false);
    custom_msgs::Box2DArray temp;
    obstacleCallback(temp);
  }
}

void RoadObstacleDetector::passiveTimerCallback(const ros::TimerEvent& time)
{
  offset_value_.data = right_lane_offset_;
  offset_pub_.publish(offset_value_);
}

bool RoadObstacleDetector::switchToActive(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("lane control set to active");
  if (state_ != EnumLaneControl::PASSIVE_RIGHT && state_ != EnumLaneControl::UNINITIALIZED)
  {
    ROS_WARN("Switched to active when node is active");
    return false;
  }
  obstacles_sub_ = nh_.subscribe("/obstacles", 1, &RoadObstacleDetector::obstacleCallback, this);
  road_lines_sub_ = nh_.subscribe("/road_lines", 1, &RoadObstacleDetector::roadLinesCallback, this);
  motion_sub_ = nh_.subscribe("selfie_out/motion", 1, &RoadObstacleDetector::motionCallback, this);
  overtaking_sub_ = nh_.subscribe("/can_overtake", 1, &RoadObstacleDetector::overtakingCallback, this);
  sendIndicators(false, false);
  return_distance_calculated_ = false;
  proof_slowdown_ = 0;
  timer_.stop();
  updateState(EnumLaneControl::ON_RIGHT);
  ROS_INFO("Lane control active mode");
  return true;
}

bool RoadObstacleDetector::switchToPassive(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("lane control set to passive");
  road_lines_sub_.shutdown();
  obstacles_sub_.shutdown();
  motion_sub_.shutdown();
  offset_value_.data = right_lane_offset_;
  speed_message_.data = max_speed_;
  offset_pub_.publish(offset_value_);
  speed_pub_.publish(speed_message_);
  updateState(EnumLaneControl::PASSIVE_RIGHT);
  timer_.start();
  ROS_INFO("Lane control passive mode");
  return true;
}

bool RoadObstacleDetector::resetNode(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Lane control reset");
  if (state_ == EnumLaneControl::PASSIVE_RIGHT)
  {
    switchToPassive(request, response);
    switchToActive(request, response);
  }

  return true;
}

void RoadObstacleDetector::sendIndicators(bool is_left_on, bool is_right_on)
{
  custom_msgs::Indicators msg;
  if (is_left_on)
  {
    msg.is_active_left = true;
  }
  else
  {
    msg.is_active_left = false;
  }

  if (is_right_on)
  {
    msg.is_active_right = true;
  }
  else
  {
    msg.is_active_right = false;
  }
  indicators_pub_.publish(msg);
}

void RoadObstacleDetector::reconfigureCB(const lane_pilot::LaneControllerConfig& config, uint32_t level)
{
  if (left_lane_offset_ != static_cast<float>(config.left_lane_offset))
  {
    left_lane_offset_ = config.left_lane_offset;
    ROS_INFO("Left lane offset new value: %f", left_lane_offset_);
  }
  if (max_distance_to_obstacle_ != static_cast<float>(config.max_distance_to_obstacle))
  {
    max_distance_to_obstacle_ = config.max_distance_to_obstacle;
    ROS_INFO("max_distance_to_obstacle new value: %f", max_distance_to_obstacle_);
  }
  if (max_length_of_obstacle_ != static_cast<float>(config.max_length_of_obstacle))
  {
    max_length_of_obstacle_ = config.max_length_of_obstacle;
    ROS_INFO("max_length_of_obstacle new value: %f", max_length_of_obstacle_);
  }
  if (max_speed_ != static_cast<float>(config.maximum_speed))
  {
    max_speed_ = config.maximum_speed;
    ROS_INFO("max_speed new value: %f", max_speed_);
  }
  if (num_corners_to_detect_ != config.num_corners_to_detect)
  {
    num_corners_to_detect_ = config.num_corners_to_detect;
    ROS_INFO("num_corners_to_detect new value: %d", num_corners_to_detect_);
  }
  if (right_lane_offset_ != static_cast<float>(config.right_lane_offset))
  {
    right_lane_offset_ = config.right_lane_offset;
    ROS_INFO("right_lane_offset new value: %f", right_lane_offset_);
  }
  if (safety_margin_ != static_cast<float>(config.safety_margin))
  {
    safety_margin_ = config.safety_margin;
    ROS_INFO("safety_margin new value: %f", safety_margin_);
  }
  if (slowdown_speed_ != static_cast<float>(config.slowdown_speed))
  {
    slowdown_speed_ = config.slowdown_speed;
    ROS_INFO("slowdown_speed new value: %f", slowdown_speed_);
  }
  if (lane_change_distance_ != static_cast<float>(config.lane_change_distance))
  {
    lane_change_distance_ = config.lane_change_distance;
    ROS_INFO("lane_change_distance new value: %f", lane_change_distance_);
  }
  if (num_proof_to_slowdown_ != static_cast<int>(config.num_proof_to_slowdown))
  {
    num_proof_to_slowdown_ = config.num_proof_to_slowdown;
    num_proof_to_return_ = num_proof_to_slowdown_;
    ROS_INFO("num_proof_to_slowdown new value: %d", num_proof_to_slowdown_);
  }
  bool ROI_changed = false;
  if (ROI_min_x_ != static_cast<float>(config.ROI_min_x))
  {
    ROI_changed = true;
    ROI_min_x_ = config.ROI_min_x;
    ROS_INFO("ROI_min_x new value: %lf", ROI_min_x_);
  }
  if (ROI_max_x_ != static_cast<float>(config.ROI_max_x))
  {
    ROI_changed = true;
    ROI_max_x_ = config.ROI_max_x;
    ROS_INFO("ROI_max_x new value: %lf", ROI_max_x_);
  }
  if (ROI_min_y_ != static_cast<float>(config.ROI_min_y))
  {
    ROI_changed = true;
    ROI_min_y_ = config.ROI_min_y;
    ROS_INFO("ROI_min_y new value: %lf", ROI_min_y_);
  }
  if (ROI_max_y_ != static_cast<float>(config.ROI_max_y))
  {
    ROI_changed = true;
    ROI_max_y_ = config.ROI_max_y;
    ROS_INFO("ROI_max_y new value: %lf", ROI_max_y_);
  }

  if (ROI_changed)
  {
    geometry_msgs::Point p;
    p.x = ROI_min_x_;
    p.y = ROI_min_y_;
    area_of_interest_box_.br = p;
    p.x = ROI_min_x_;
    p.y = ROI_max_y_;
    area_of_interest_box_.bl = p;
    p.x = ROI_max_x_;
    p.y = ROI_min_y_;
    area_of_interest_box_.tr = p;
    p.x = ROI_max_x_;
    p.y = ROI_max_y_;
    area_of_interest_box_.tl = p;
  }

  bool right_obst_area_changed = false;
  if (right_obst_area_min_x_ != static_cast<float>(config.right_obst_area_min_x))
  {
    right_obst_area_changed = true;
    right_obst_area_min_x_ = config.right_obst_area_min_x;
    ROS_INFO("right_obst_area_min_x new value: %lf", right_obst_area_min_x_);
  }
  if (right_obst_area_max_x_ != static_cast<float>(config.right_obst_area_max_x))
  {
    right_obst_area_changed = true;
    right_obst_area_max_x_ = config.right_obst_area_max_x;
    ROS_INFO("right_obst_area_max_x new value: %lf", right_obst_area_max_x_);
  }
  if (right_obst_area_min_y_ != static_cast<float>(config.right_obst_area_min_y))
  {
    right_obst_area_changed = true;
    right_obst_area_min_y_ = config.right_obst_area_min_y;
    ROS_INFO("right_obst_area_min_y new value: %lf", right_obst_area_min_y_);
  }
  if (right_obst_area_max_y_ != static_cast<float>(config.right_obst_area_max_y))
  {
    right_obst_area_changed = true;
    right_obst_area_max_y_ = config.right_obst_area_max_y;
    ROS_INFO("right_obst_area_max_y new value: %lf", right_obst_area_max_y_);
  }

  if (right_obst_area_changed)
  {
    geometry_msgs::Point p;
    p.x = right_obst_area_min_x_;
    p.y = right_obst_area_min_y_;
    right_obst_area_box_.br = p;
    p.x = right_obst_area_min_x_;
    p.y = right_obst_area_max_y_;
    right_obst_area_box_.bl = p;
    p.x = right_obst_area_max_x_;
    p.y = right_obst_area_min_y_;
    right_obst_area_box_.tr = p;
    p.x = right_obst_area_max_x_;
    p.y = right_obst_area_max_y_;
    right_obst_area_box_.tl = p;
  }

  if (lane_change_speed_ != static_cast<float>(config.lane_change_speed))
  {
    lane_change_speed_ = config.lane_change_speed;
    ROS_INFO("lane_change_speed new value: %lf", lane_change_speed_);
  }
}
