/**
 *Copyright ( c ) 2019, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#include <list>
#include <string>
#include <algorithm>
#include <vector>

#include <common/obstacle_box.h>

Point::Point() : x(0), y(0)
{
}

Point::Point(float x_, float y_) : x(x_), y(y_)
{
}
Point::Point(geometry_msgs::Point32 point)
{
  x = point.x;
  y = point.y;
}
Point Point::operator=(const geometry_msgs::Point32 other)
{
  this->x = other.x;
  this->y = other.y;
  return *this;
}

Point Point::operator=(const geometry_msgs::Point other)
{
  this->x = other.x;
  this->y = other.y;
  return *this;
}

void Point::reset()
{
  x = 0;
  y = 0;
}

bool Point::check_position(float min_x, float max_x, float min_y, float max_y)
{
  if (x < min_x || x > max_x || y < min_y || y > max_y)
    return 0;
  else
    return 1;
}

Box::Box(const Box& other)
{
  bottom_left = other.bottom_left;
  bottom_right = other.bottom_right;
  top_left = other.top_left;
  top_right = other.top_right;
  left_vertical_line = other.left_vertical_line;
  bottom_horizontal_line = other.bottom_horizontal_line;
}

Box::Box(const float x_min, const float x_max, const float y_min, const float y_max)
{
  bottom_left = Point(x_min, y_max);
  bottom_right = Point(x_min, y_min);
  top_left = Point(x_max, y_max);
  top_right = Point(x_max, y_min);

  bottom_horizontal_line.a = 0;
  bottom_horizontal_line.b = x_min;
}

Box::Box(geometry_msgs::Polygon right_side_poly)
{
  bool is_in_the_front = true;
  float avg_x = (right_side_poly.points[0].x + right_side_poly.points[1].x + right_side_poly.points[2].x +
                 right_side_poly.points[3].x) /
                4;
  if (avg_x < 0)
    is_in_the_front = false;

  Point zero(0, 0);
  vector<float> distances =
  {
    zero.get_distance(right_side_poly.points[0]), zero.get_distance(right_side_poly.points[1]),
    zero.get_distance(right_side_poly.points[2]), zero.get_distance(right_side_poly.points[3])
  };

  vector<float>::iterator min;

  min = min_element(distances.begin(), distances.end());
  if (is_in_the_front)
    bottom_left = right_side_poly.points[std::distance(distances.begin(), min)];
  else
    top_left = right_side_poly.points[std::distance(distances.begin(), min)];
  *min = 1000;
  //    cout <<"distance from the nearest point of the box: " <<
  //    bottom_left.x << endl;

  // pick two closest points
  Point a, b;
  min = min_element(distances.begin(), distances.end());
  a = right_side_poly.points[std::distance(distances.begin(), min)];
  *min = 1000;
  min = min_element(distances.begin(), distances.end());
  b = right_side_poly.points[std::distance(distances.begin(), min)];
  *min = 1000;
  // pick the one with smaller y coordinate -> it's the left one
  if (a.x > b.x)
  {
    if (is_in_the_front)
    {
      top_left = a;
      bottom_right = b;
    }
    else
    {
      bottom_left = a;
      top_right = b;
    }
  }
  else
  {
    if (is_in_the_front)
    {
      top_left = b;
      bottom_right = a;
    }
    else
    {
      bottom_left = b;
      top_right = a;
    }
  }
  min = min_element(distances.begin(), distances.end());
  if (is_in_the_front)
    top_right = right_side_poly.points[std::distance(distances.begin(), min)];
  else
    bottom_right = right_side_poly.points[std::distance(distances.begin(), min)];
}

Box::Box(Point b_l, Point b_r, Point t_l, Point t_r)
  : bottom_left(b_l), bottom_right(b_r), top_left(t_l), top_right(t_r)
{
  make_lines();
}

void Box::reset()
{
  bottom_left.reset();
  top_left.reset();
  top_right.reset();
  bottom_right.reset();
  left_vertical_line.a = 0;
  left_vertical_line.b = 0;
  bottom_horizontal_line.a = 0;
  bottom_horizontal_line.b = 0;
}

void Box::make_poly(geometry_msgs::Polygon& poly)
{
  geometry_msgs::Point32 top_l;
  top_l.x = top_left.x;
  top_l.y = top_left.y;
  top_l.z = 0;
  geometry_msgs::Point32 top_r;
  top_r.x = top_right.x;
  top_r.y = top_right.y;
  top_r.z = 0;
  geometry_msgs::Point32 bottom_r;
  bottom_r.x = bottom_right.x;
  bottom_r.y = bottom_right.y;
  bottom_r.z = 0;
  geometry_msgs::Point32 bottom_l;
  bottom_l.x = bottom_left.x;
  bottom_l.y = bottom_left.y;
  bottom_l.z = 0;

  poly.points = {top_l, bottom_l, bottom_r, top_r};
}

void Box::make_lines()
{
  left_vertical_line.a = (top_left.y - bottom_left.y) / (top_left.x - bottom_left.x);
  left_vertical_line.b = top_left.y - left_vertical_line.a * top_left.x;

  bottom_horizontal_line.a = (bottom_right.y - bottom_left.y) / (bottom_right.x - bottom_left.x);
  bottom_horizontal_line.b = bottom_right.y - bottom_horizontal_line.a * bottom_right.x;
}

void Box::visualize(const ros::Publisher& pub, const std::string& name, float red, float green, float blue,
                    int lifetime)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = name;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  if (lifetime != 0)
    marker.lifetime = ros::Duration(lifetime);
  else
    marker.lifetime = ros::Duration();

  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = 1.0f;

  marker.scale.x = 0.01;
  marker.scale.y = 0.01;

  geometry_msgs::Point marker_point;
  marker_point.z = 0;

  pushToMarker(marker_point, marker);

  pub.publish(marker);
}

void Box::visualizeList(const std::list<Box> boxes, const ros::Publisher& pub, const std::string& name, float red,
                        float green, float blue)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = name;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  marker.lifetime = ros::Duration(2);

  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = 1.0f;

  marker.scale.x = 0.01;
  marker.scale.y = 0.01;

  geometry_msgs::Point marker_point;
  marker_point.z = 0;

  for (list<Box>::const_iterator iter = boxes.begin(); iter != boxes.end(); iter++)
  {
    iter->pushToMarker(marker_point, marker);
  }

  pub.publish(marker);
}

void Box::print()
{
  ROS_INFO("  bottom left: ");
  bottom_left.print();
  ROS_INFO("  bottom_right: ");
  bottom_right.print();
  ROS_INFO("  top left: ");
  top_left.print();
  ROS_INFO("  top right: ");
  top_right.print();
}
void Box::print_box_dimensions()
{
  float bottom_edge = bottom_left.get_distance(bottom_right);
  float top_edge = top_left.get_distance(top_right);
  float right_edge = top_right.get_distance(bottom_right);
  float left_edge = top_left.get_distance(bottom_left);

  ROS_INFO("bottom_edge: %f", bottom_edge);
  ROS_INFO("top_edge: %f", top_edge);
  ROS_INFO("right_edge:  %f", right_edge);
  ROS_INFO("left_edge: %f", left_edge);
}

void Box::pushToMarker(geometry_msgs::Point& marker_point, visualization_msgs::Marker& marker) const
{
  marker_point.x = this->bottom_left.x;
  marker_point.y = this->bottom_left.y;
  marker.points.push_back(marker_point);
  marker_point.x = this->bottom_right.x;
  marker_point.y = this->bottom_right.y;
  marker.points.push_back(marker_point);

  marker_point.x = this->bottom_right.x;
  marker_point.y = this->bottom_right.y;
  marker.points.push_back(marker_point);
  marker_point.x = this->top_right.x;
  marker_point.y = this->top_right.y;
  marker.points.push_back(marker_point);

  marker_point.x = this->top_right.x;
  marker_point.y = this->top_right.y;
  marker.points.push_back(marker_point);
  marker_point.x = this->top_left.x;
  marker_point.y = this->top_left.y;
  marker.points.push_back(marker_point);

  marker_point.x = this->top_left.x;
  marker_point.y = this->top_left.y;
  marker.points.push_back(marker_point);
  marker_point.x = this->bottom_left.x;
  marker_point.y = this->bottom_left.y;
  marker.points.push_back(marker_point);
}
