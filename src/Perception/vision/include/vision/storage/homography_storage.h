/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef VISION_STORAGE_HOMOGRAPHY_STORAGE_H
#define VISION_STORAGE_HOMOGRAPHY_STORAGE_H
#include <vision/storage/unit_storage_interface.h>
#include <vision/storage/global_storage.h>
#include <opencv2/imgproc/imgproc.hpp>

class HomographyStorage : UnitStorageInterface
{
public: 
  cv::Mat &image_rect_;
  cv::Mat &homography_frame_;
  HomographyStorage(GlobalStorage &global_storage_);
};


#endif  // VISION_STORAGE_HOMOGRAPHY_STORAGE_H
