/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef NEW_PERCEPTION_INTERSECTIONPATCH_H
#define NEW_PERCEPTION_INTERSECTIONPATCH_H
#include "new_perception/Patch.h"
#include <utility>

class IntersectionPatch : public Patch
{
public:
  IntersectionPatch(TypePatch type, geometry_msgs::Pose2D pose, std::pair<double, double> dimension);
  ~IntersectionPatch() {}
};

IntersectionPatch::IntersectionPatch(TypePatch type, geometry_msgs::Pose2D pose, std::pair<double, double> dimension)
    : Patch(type, pose, dimension)
{
  std::cout << "IntersectionPatch constructor\n";
}

#endif /* NEW_PERCEPTION_INTERSECTIONPATCH_H */
