/*
* Copyright ( c ) 2019, KNR Selfie
* This code is licensed under BSD license (see LICENSE for details)
**/

#pragma once
#ifndef OBSTACLE_DETECTOR_OBSTACLES_GENERATOR_H
#define OBSTACLE_DETECTOR_OBSTACLES_GENERATOR_H

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <algorithm>
#include <functional>

#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>
#include <obstacle_detector/ObstacleDetectorConfig.h>
#include <custom_msgs/Box2DArray.h>
#include <custom_msgs/Box2D.h>

struct Point
{
  float x;
  float y;
};

struct Line
{
  Point start_point;
  Point end_point;
  float slope;
  float b;
  float a;
  float length;
};

class ObstaclesGenerator
{
public:
  ObstaclesGenerator(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~ObstaclesGenerator();
  bool init();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber scan_sub_;
  ros::Publisher obstacles_pub_;
  ros::Publisher visualization_lines_pub_;
  ros::Publisher visualization_obstacles_pub_;
  tf::TransformListener transformListener_;

  tf::StampedTransform transform_;

  dynamic_reconfigure::Server<obstacle_detector::ObstacleDetectorConfig> dr_server_;
  dynamic_reconfigure::Server<obstacle_detector::ObstacleDetectorConfig>::CallbackType dr_server_CB_;
  void reconfigureCB(obstacle_detector::ObstacleDetectorConfig& config, uint32_t level);

  std::vector<Line> line_array_;
  std::vector<std::vector<Point>> segments_;
  custom_msgs::Box2DArray obstacle_array_;
  sensor_msgs::LaserScan scan;
  sensor_msgs::LaserScan scan_;
  void laserScanCallback(const sensor_msgs::LaserScan& msg);
  void divideIntoSegments();
  void generateLines();
  Point getXY(float angle, float range);
  float getSlope(Point& p1, Point& p2);
  float getA(Point& p1, Point& p2);
  float getDistance(Point& p1, Point& p2);
  float getDistance(geometry_msgs::Point& p1, geometry_msgs::Point& p2);
  void visualizeLines();
  void visualizeObstacles();
  void printInfoParams();
  void generateObstacles();
  void convertUpsideDown();
  void convertToOutputFrame();
  void initializeTransform();
  void transformPoint(geometry_msgs::Point&);

  bool visualization_;
  float max_range_;
  float min_range_;
  std::string visualization_frame_;
  std::string obstacles_frame_;
  std::string output_frame_;
  float segment_threshold_;
  float min_segment_size_;
  float max_segment_size_;
  float min_to_divide_;
};

#endif  // OBSTACLE_DETECTOR_OBSTACLES_GENERATOR_H
