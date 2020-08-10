/**
 *Copyright ( c ) 2019, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef COMMON_OBSTACLE_BOX_H
#define COMMON_OBSTACLE_BOX_H

#include <iostream>
#include <string>
#include <list>

#include <custom_msgs/PolygonArray.h>
#include <geometry_msgs/Polygon.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>

using std::list;
using std::vector;

class Point
{
public:
  float x;
  float y;

  Point();
  Point(float x_, float y_);
  explicit Point(geometry_msgs::Point32 point);
  explicit Point(geometry_msgs::Point point);
  ~Point()
  {
  }
  Point operator=(const geometry_msgs::Point32 other);
  Point operator=(const geometry_msgs::Point other);

  void reset();

  bool check_position(float min_x, float max_x, float min_y, float max_y);
  float get_distance(const Point other)
  {
    return (std::sqrt(std::pow(other.x - x, 2) + pow(other.y - y, 2)));
  }
  float get_distance(geometry_msgs::Point32 other)
  {
    return (std::sqrt(std::pow(other.x - x, 2) + pow(other.y - y, 2)));
  }
  void print()
  {
    ROS_INFO("(x,y) = ( %f, %f )", x, y);
  }
};

// y= ax+b
struct Line
{
  float a;
  float b;
};

// local coordinates
class Box
{
public:
  Point bottom_left;
  Point top_left;
  Point top_right;
  Point bottom_right;
  Line left_vertical_line;
  Line bottom_horizontal_line;

public:
  Box()
  {
  }
  Box(const Box& other);
  Box(const float x_min, const float x_max, const float y_min, const float y_max);

  // this constructor finds and assign correct points to corners of our box
  explicit Box(geometry_msgs::Polygon right_side_poly);

  Box(Point b_l, Point b_r, Point t_l, Point t_r);
  void reset();

  void make_poly(geometry_msgs::Polygon& poly);

  void make_lines();

  void visualize(const ros::Publisher& pub, const std::string& name, float red = 0.4, float green = 0.3, float blue = 0,
                 int lifetime = 2);
  void visualizeList(const std::list<Box> boxes, const ros::Publisher& pub, const std::string& name, float red = 0.4,
                     float green = 0.3, float blue = 0);

  void print();
  void print_box_dimensions();

private:
  void pushToMarker(geometry_msgs::Point& marker_point, visualization_msgs::Marker& marker) const;
};

#endif  // COMMON_OBSTACLE_BOX_H
