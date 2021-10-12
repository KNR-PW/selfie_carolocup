/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef NEW_PERCEPTION_PATCHDISTRIBUTION_H
#define NEW_PERCEPTION_PATCHDISTRIBUTION_H
#include "nav_msgs/OccupancyGrid.h"
#include "Feature.h"
#include "Patch.h"
#include <vector>

class PatchDistribution
{
protected:
  nav_msgs::OccupancyGrid distribution;

public:
  PatchDistribution();
  virtual ~PatchDistribution() {}
  virtual void distribute(std::vector<Feature> features, std::vector<Patch> patches);

  nav_msgs::OccupancyGrid getDistribution();
};

PatchDistribution::PatchDistribution()
{
}

nav_msgs::OccupancyGrid PatchDistribution::getDistribution()
{
  return this->distribution;
}

#endif /* NEW_PERCEPTION_PATCHDISTRIBUTION_H */
