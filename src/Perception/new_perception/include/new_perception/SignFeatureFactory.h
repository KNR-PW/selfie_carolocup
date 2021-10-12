/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef NEW_PERCEPTION_SIGNFEATUREFACTORY_H
#define NEW_PERCEPTION_SIGNFEATUREFACTORY_H
#include "new_perception/FeatureFactory.h"
#include "new_perception/SignFeature.h"

class SignFeatureFactory : public FeatureFactory
{
public:
  SignFeatureFactory();

  void produce(const sensor_msgs::ImageConstPtr &msg) override;
  ~SignFeatureFactory() {}
};

SignFeatureFactory::SignFeatureFactory()
{
  std::cout << "Sign Feature Factory constructor!\n";
};

void SignFeatureFactory::produce(const sensor_msgs::ImageConstPtr &msg)
{
  geometry_msgs::Pose2D pose2d;
};
#endif /* NEW_PERCEPTION_SIGNFEATUREFACTORY_H */
