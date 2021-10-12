/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef NEW_PERCEPTION_FEATUREFACTORY_H
#define NEW_PERCEPTION_FEATUREFACTORY_H
#include "sensor_msgs/Image.h"
#include "Feature.h"
#include <vector>

class FeatureFactory
{
protected:
  std::vector<Feature> features;

public:
  FeatureFactory();
  std::vector<Feature> getFeatures();

  virtual ~FeatureFactory() {}
  virtual void produce(const sensor_msgs::ImageConstPtr &msg) = 0;
};

FeatureFactory::FeatureFactory()
{
  std::cout << "Abstract Feature Factory constructor!\n";
}

std::vector<Feature> FeatureFactory::getFeatures()
{
  return this->features;
}
#endif /* NEW_PERCEPTION_FEATUREFACTORY_H */
