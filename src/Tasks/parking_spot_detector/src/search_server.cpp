/**
 *Copyright ( c ) 2019, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#include <parking_spot_detector/search_server.hpp>

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
  pnh_.param<float>("ROI_min_x", ROI_min_x_, 0.01);
  pnh_.param<float>("ROI_max_x", ROI_max_x_, 2);
  pnh_.param<float>("ROI_min_y", ROI_min_y_, -1);
  pnh_.param<float>("ROI_max_y", ROI_max_y_, 0.2);
  pnh_.param<float>("default_speed_in_parking_zone", default_speed_in_parking_zone, 0.9);
  pnh_.param<float>("speed_when_found_place", speed_when_found_place, 0.3);
  pnh_.param<bool>("visualization_in_searching", visualization, true);
  pnh_.param<float>("max_distance_to_free_place", max_distance_to_free_place_, 0.8);
  pnh_.param<float>("box_angle_deg", tangens_of_box_angle_, 55);  // maximum angle between car and found place
  pnh_.param<float>("length_of_parking_area", length_of_parking_area_, 5.5);
  tangens_of_box_angle_ = tan(tangens_of_box_angle_ * M_PI / 180);

  speed_pub_ = nh_.advertise<std_msgs::Float64>("/max_speed", 5);

  speed_current.data = default_speed_in_parking_zone;
  if (visualization)
  {
    visualizator_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization/free_place", 1);
  }
}

SearchServer::~SearchServer()
{
}

bool SearchServer::init()
{
  obstacles_sub_ = nh_.subscribe("/obstacles", 1, &SearchServer::manager, this);
  distance_sub_ = nh_.subscribe("/selfie_out/motion", 1, &SearchServer::distanceCb, this);

  speed_current.data = default_speed_in_parking_zone;
  speed_pub_.publish(speed_current);
  min_spot_lenght = search_server_.acceptNewGoal()->min_spot_lenght;
  updateState(selfie::SEARCHING_PARKING_SPOT);
  ROS_INFO("Initialized");
  return true;
}

void SearchServer::manager(const custom_msgs::Box2DArray& msg)
{
  // to save cpu time just do nothing when new scan comes
  if (!search_server_.isActive())
  {
    ROS_INFO("parking_spot_detector is not active");
    return;
  }
  filter_boxes(msg);
  if (visualization)
  {
    selfie::visualizeBoxes2D(boxes_on_the_right_side, visualizator_pub_, "FilteredBoxes", 0, 255, 1);
    selfie::visualizeBox2D(ROI_min_x_, ROI_max_x_, ROI_min_y_, ROI_max_y_, visualizator_pub_, "ROI", 1, 1, 1);
  }

  switch (state_)
  {
    case selfie::SEARCHING_PARKING_SPOT:
      if (find_free_places())
      {
        updateState(selfie::PLACE_INITIALLY_FOUND);
        speed_current.data = speed_when_found_place;
        speed_pub_.publish(speed_current);
      }
      break;

    case selfie::PLACE_INITIALLY_FOUND:
      if (find_free_places())
      {
        if (first_free_place.bl.x <= max_distance_to_free_place_)
        {
          updateState(selfie::PLACE_PROPER_FOUND);
          speed_current.data = default_speed_in_parking_zone;
        }
        speed_pub_.publish(speed_current);
      }
      else
      {
        speed_current.data = default_speed_in_parking_zone;
        updateState(selfie::SEARCHING_PARKING_SPOT);
      }
      speed_pub_.publish(speed_current);
      break;

    case selfie::PLACE_PROPER_FOUND:
      if (find_free_places())
      {
        std::cout << "Found proper place\nsending result";
        send_goal();
      }
      else
      {
        std::cout << "Place lost\n";
        speed_current.data = default_speed_in_parking_zone;
        updateState(selfie::SEARCHING_PARKING_SPOT);
        speed_pub_.publish(speed_current);
      }
      break;

    default:
      ROS_INFO("Err, wrong action_status");
      break;
  }
}

void SearchServer::filter_boxes(const custom_msgs::Box2DArray& msg)
{
  boxes_on_the_right_side.clear();
  if (msg.boxes.empty())
  {
    return;
  }

  for (int box_nr = msg.boxes.size() - 1; box_nr >= 0; box_nr--)
  {
    bool box_inside = false;
    // check corners of obstacle
    if (isPointInsideROI(msg.boxes[box_nr].tl))
      box_inside = true;
    else if (isPointInsideROI(msg.boxes[box_nr].tr))
      box_inside = true;
    else if (isPointInsideROI(msg.boxes[box_nr].bl))
      box_inside = true;
    else if (isPointInsideROI(msg.boxes[box_nr].br))
      box_inside = true;

    if (box_inside)
    {
      float left_vertical_line_a =
          (msg.boxes[box_nr].tl.y - msg.boxes[box_nr].bl.y) / (msg.boxes[box_nr].tl.x - msg.boxes[box_nr].bl.x);

      if (abs(left_vertical_line_a) < tangens_of_box_angle_)  // filters out boxes which are not parallel to car
      {
        boxes_on_the_right_side.insert(boxes_on_the_right_side.begin(), msg.boxes[box_nr]);
      }
    }
  }
}

bool SearchServer::isPointInsideROI(const geometry_msgs::Point& p)
{
  return !(p.x < ROI_min_x_ || p.x > ROI_max_x_ || p.y < ROI_min_y_ || p.y > ROI_max_y_);
}

bool SearchServer::find_free_places()
{
  if (boxes_on_the_right_side.size() < 2 || boxes_on_the_right_side.empty())
  {
    return false;
  }
  float min_space = min_spot_lenght;

  vector<custom_msgs::Box2D>::iterator iter = boxes_on_the_right_side.begin();
  vector<custom_msgs::Box2D>::const_iterator end_iter = boxes_on_the_right_side.cend();
  for (; iter + 1 != end_iter; ++iter)
  {
    float dist = getDistance((*iter).tl, (*(iter + 1)).bl);
    // ROS_INFO("dist: %f", dist);
    if (dist > min_space)
    {
      custom_msgs::Box2D tmp_box;
      tmp_box.bl = (*iter).tl;
      tmp_box.br = (*iter).tr;
      tmp_box.tl = (*(iter + 1)).bl;
      tmp_box.tr = (*(iter + 1)).br;

      geometry_msgs::Point point_centroid;
      point_centroid.x = (tmp_box.bl.x + tmp_box.br.x + tmp_box.tr.x + tmp_box.tl.x) / 4.0;
      point_centroid.y = (tmp_box.bl.y + tmp_box.br.y + tmp_box.tr.y + tmp_box.tl.y) / 4.0;
      tmp_box.point_centroid = point_centroid;

      tmp_box.width = getDistance(tmp_box.tr, tmp_box.tl);
      tmp_box.length = getDistance(tmp_box.tr, tmp_box.br);

      first_free_place = tmp_box;
      ROS_INFO(
          "Found place \nTL: x=%lf y=%lf\nTR: x=%lf y=%lf\nBL x=%lf "
          "y=%lf\nBR x=%lf y=%lf\n",
          tmp_box.tl.x,
          tmp_box.tl.y,
          tmp_box.tr.x,
          tmp_box.tr.y,
          tmp_box.bl.x,
          tmp_box.bl.y,
          tmp_box.br.x,
          tmp_box.br.y);
      if (visualization)
      {
        selfie::visualizeBox2D(tmp_box, visualizator_pub_, "first_free_place", 100, 255, 200);
      }
      return true;
    }
  }
  return false;
}

float SearchServer::getDistance(geometry_msgs::Point& p1, geometry_msgs::Point& p2)
{
  float dx = p2.x - p1.x;
  float dy = p2.y - p1.y;
  return std::sqrt(dx * dx + dy * dy);
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

void SearchServer::send_goal()
{
  result.parking_spot = first_free_place;
  ROS_INFO("Place found and sent");
  search_server_.setSucceeded(result);
  endAction();
}

void SearchServer::updateState(const int& state)
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
  obstacles_sub_.shutdown();
  distance_sub_.shutdown();
  max_distance_calculated_ = false;
}

void SearchServer::reconfigureCB(const parking_spot_detector::DetectParkingSpotConfig& config, uint32_t level)
{
  if (default_speed_in_parking_zone != static_cast<float>(config.default_speed_in_parking_zone))
  {
    default_speed_in_parking_zone = config.default_speed_in_parking_zone;
    ROS_INFO("default_speed in parking_zone new value: %f", default_speed_in_parking_zone);
  }
  if (max_distance_to_free_place_ != static_cast<float>(config.max_distance_to_free_place))
  {
    max_distance_to_free_place_ = config.max_distance_to_free_place;
    ROS_INFO("max_distance_to_free_place_ new value: %f", max_distance_to_free_place_);
  }
  if (speed_when_found_place != static_cast<float>(config.speed_when_found_place))
  {
    speed_when_found_place = config.speed_when_found_place;
    ROS_INFO("speed_when_found_place new value: %f", speed_when_found_place);
  }
}
