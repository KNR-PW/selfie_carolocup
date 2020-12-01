/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef VISION_STORAGE_GLOBAL_STORAGE_H
#define VISION_STORAGE_GLOBAL_STORAGE_H
#include <opencv2/imgproc/imgproc.hpp>

class GlobalStorage
{
public:
  cv::Mat image_rect_;
  cv::Mat homography_frame_;
};
#endif  // VISION_STORAGE_GLOBAL_STORAGE_H
