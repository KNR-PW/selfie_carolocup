/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef NEW_PERCEPTION_PATCH_H
#define NEW_PERCEPTION_PATCH_H
#include "TypePatch.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include <utility>

class Patch
{
protected:
  const TypePatch type;
  const std::pair<double, double> dimension;
  geometry_msgs::Pose2D pose;

public:
  Patch(TypePatch type, geometry_msgs::Pose2D pose, std::pair<double, double> dimension);
  virtual ~Patch() {}

  TypePatch getType();
  geometry_msgs::Pose2D getPose();
  std::pair<double, double> getDimension();
  void updatePose(nav_msgs::Odometry odometry);
};

Patch::Patch(TypePatch type, geometry_msgs::Pose2D pose, std::pair<double, double> dimension)
    : type(type), pose(pose), dimension(dimension)
{
  std::cout << "Patch constructor\n";
}

#endif /* NEW_PERCEPTION_PATCH_H */
