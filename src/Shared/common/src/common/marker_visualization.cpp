/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#include <string>
#include <vector>
#include <list>

#include <common/marker_visualization.h>

namespace selfie
{
visualization_msgs::Marker
constructMarkerBase(const std::string& name, float r, float g, float b, int lifetime, const std::string& frame_id)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = name;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;

  if (lifetime != 0)
    marker.lifetime = ros::Duration(lifetime);
  else
    marker.lifetime = ros::Duration();

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0f;

  marker.scale.x = 0.01;

  return marker;
}

void visualizeBox2D(const float& min_x,
                    const float& max_x,
                    const float& min_y,
                    const float& max_y,
                    const ros::Publisher& pub,
                    const std::string& name,
                    float r,
                    float g,
                    float b,
                    int lifetime,
                    const std::string& frame_id)
{
  visualization_msgs::Marker marker = constructMarkerBase(name, r, g, b, lifetime, frame_id);

  geometry_msgs::Point marker_point;
  marker_point.z = 0;

  marker_point.x = min_x;
  marker_point.y = min_y;
  marker.points.push_back(marker_point);
  marker_point.x = min_x;
  marker_point.y = max_y;
  marker.points.push_back(marker_point);

  marker_point.x = min_x;
  marker_point.y = max_y;
  marker.points.push_back(marker_point);
  marker_point.x = max_x;
  marker_point.y = max_y;
  marker.points.push_back(marker_point);

  marker_point.x = max_x;
  marker_point.y = max_y;
  marker.points.push_back(marker_point);
  marker_point.x = max_x;
  marker_point.y = min_y;
  marker.points.push_back(marker_point);

  marker_point.x = max_x;
  marker_point.y = min_y;
  marker.points.push_back(marker_point);
  marker_point.x = min_x;
  marker_point.y = min_y;
  marker.points.push_back(marker_point);

  pub.publish(marker);
}

void visualizeBox2D(const custom_msgs::Box2D& box,
                    const ros::Publisher& pub,
                    const std::string& name,
                    float r,
                    float g,
                    float b,
                    int lifetime,
                    const std::string& frame_id)
{
  visualization_msgs::Marker marker = constructMarkerBase(name, r, g, b, lifetime, frame_id);

  marker.points.push_back(box.tl);
  marker.points.push_back(box.tr);

  marker.points.push_back(box.tr);
  marker.points.push_back(box.br);

  marker.points.push_back(box.br);
  marker.points.push_back(box.bl);

  marker.points.push_back(box.bl);
  marker.points.push_back(box.tl);

  pub.publish(marker);
}

void visualizeBoxes2D(const std::vector<custom_msgs::Box2D>& boxes,
                      const ros::Publisher& pub,
                      const std::string& name,
                      float r,
                      float g,
                      float b,
                      int lifetime,
                      const std::string& frame_id)
{
  if (boxes.empty())
    return;

  visualization_msgs::Marker marker = constructMarkerBase(name, r, g, b, lifetime, frame_id);

  for (auto& box : boxes)
  {
    marker.points.push_back(box.tl);
    marker.points.push_back(box.tr);

    marker.points.push_back(box.tr);
    marker.points.push_back(box.br);

    marker.points.push_back(box.br);
    marker.points.push_back(box.bl);

    marker.points.push_back(box.bl);
    marker.points.push_back(box.tl);
  }

  pub.publish(marker);
}

void visualizeBoxes2D(const std::list<custom_msgs::Box2D>& boxes,
                      const ros::Publisher& pub,
                      const std::string& name,
                      float r,
                      float g,
                      float b,
                      int lifetime,
                      const std::string& frame_id)
{
  if (boxes.empty())
    return;

  visualization_msgs::Marker marker = constructMarkerBase(name, r, g, b, lifetime, frame_id);

  for (auto& box : boxes)
  {
    marker.points.push_back(box.tl);
    marker.points.push_back(box.tr);

    marker.points.push_back(box.tr);
    marker.points.push_back(box.br);

    marker.points.push_back(box.br);
    marker.points.push_back(box.bl);

    marker.points.push_back(box.bl);
    marker.points.push_back(box.tl);
  }

  pub.publish(marker);
}

void visualizeBoxes2D(const custom_msgs::Box2DArray& boxes,
                      const ros::Publisher& pub,
                      const std::string& name,
                      float r,
                      float g,
                      float b,
                      int lifetime,
                      const std::string& frame_id)
{
  if (boxes.boxes.empty())
    return;

  visualization_msgs::Marker marker = constructMarkerBase(name, r, g, b, lifetime, frame_id);

  for (auto& box : boxes.boxes)
  {
    marker.points.push_back(box.tl);
    marker.points.push_back(box.tr);

    marker.points.push_back(box.tr);
    marker.points.push_back(box.br);

    marker.points.push_back(box.br);
    marker.points.push_back(box.bl);

    marker.points.push_back(box.bl);
    marker.points.push_back(box.tl);
  }

  pub.publish(marker);
}

void visualizeEmpty(const ros::Publisher& pub,
                    const std::string& name,
                    float r,
                    float g,
                    float b,
                    int lifetime,
                    const std::string& frame_id)
{
  visualization_msgs::Marker marker = constructMarkerBase(name, r, g, b, lifetime, frame_id);

  marker.points.push_back(geometry_msgs::Point());
  marker.points.push_back(geometry_msgs::Point());
  pub.publish(marker);
}
}  // namespace selfie
