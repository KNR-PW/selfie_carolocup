/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#include "new_perception/FeatureFactory.h"
#include "new_perception/SignFeatureFactory.h"
#include "new_perception/Patch.h"
#include "new_perception/IntersectionPatch.h"
#include "sensor_msgs/Image.h"
#include <iostream>
#include <utility>

int main()
{
  TypePatch type;
  geometry_msgs::Pose2D pose;
  std::pair<double, double> dimension;
  IntersectionPatch(type, pose, dimension);
  const sensor_msgs::ImageConstPtr test;
  FeatureFactory *factory = new SignFeatureFactory;
  factory->produce(test);
  factory->getFeatures();
  return 0;
}
