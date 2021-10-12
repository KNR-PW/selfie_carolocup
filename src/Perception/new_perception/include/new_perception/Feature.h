/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef NEW_PERCEPTION_FEATURE_H
#define NEW_PERCEPTION_FEATURE_H
#include "TypeFeature.h"
#include "geometry_msgs/Pose2D.h"

class Feature
{
protected:
  const TypeFeature type;
  geometry_msgs::Pose2D pose2d;

public:
  virtual ~Feature() {}
  Feature(geometry_msgs::Pose2D pose2d, TypeFeature type);
  TypeFeature getType();
  geometry_msgs::Pose2D getPose();
};

Feature::Feature(geometry_msgs::Pose2D pose2d, TypeFeature type)
    : type(type), pose2d(pose2d) {}

TypeFeature Feature::getType()
{
  return this->type;
}

geometry_msgs::Pose2D Feature::getPose()
{
  return this->pose2d;
}

#endif /* NEW_PERCEPTION_FEATURE_H */
