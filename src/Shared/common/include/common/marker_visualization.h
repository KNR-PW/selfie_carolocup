/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef COMMON_MARKER_VISUALIZATION_H
#define COMMON_MARKER_VISUALIZATION_H

#include <iostream>
#include <string>
#include <vector>
#include <list>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <custom_msgs/Box2D.h>
#include <custom_msgs/Box2DArray.h>

namespace selfie
{
visualization_msgs::Marker
constructMarkerBase(const std::string& name, float r, float g, float b, int lifetime, const std::string& frame_id);

void visualizeBox2D(float& min_x,
                    float& max_x,
                    float& min_y,
                    float& max_y,
                    const ros::Publisher& pub,
                    const std::string& name,
                    float r = 0.4,
                    float g = 0.3,
                    float b = 0,
                    int lifetime = 2,
                    const std::string& frame_id = "base_link");

void visualizeBox2D(const custom_msgs::Box2D& box,
                    const ros::Publisher& pub,
                    const std::string& name,
                    float r = 0.4,
                    float g = 0.3,
                    float b = 0,
                    int lifetime = 2,
                    const std::string& frame_id = "base_link");

void visualizeBoxes2D(const std::vector<custom_msgs::Box2D>& boxes,
                      const ros::Publisher& pub,
                      const std::string& name,
                      float r = 0.4,
                      float g = 0.3,
                      float b = 0,
                      int lifetime = 2,
                      const std::string& frame_id = "base_link");

void visualizeBoxes2D(const std::list<custom_msgs::Box2D>& boxes,
                      const ros::Publisher& pub,
                      const std::string& name,
                      float r = 0.4,
                      float g = 0.3,
                      float b = 0,
                      int lifetime = 2,
                      const std::string& frame_id = "base_link");

void visualizeBoxes2D(const custom_msgs::Box2DArray& boxes,
                      const ros::Publisher& pub,
                      const std::string& name,
                      float r = 0.4,
                      float g = 0.3,
                      float b = 0,
                      int lifetime = 2,
                      const std::string& frame_id = "base_link");

void visualizeEmpty(const ros::Publisher& pub,
                    const std::string& name,
                    float r = 0.4,
                    float g = 0.3,
                    float b = 0,
                    int lifetime = 2,
                    const std::string& frame_id = "base_link");
}  // namespace selfie
#endif  // COMMON_MARKER_VISUALIZATION_H
