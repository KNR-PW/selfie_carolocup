/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef VISION_UNITS_HOMOGRAPHY_UNIT_H
#define VISION_UNITS_HOMOGRAPHY_UNIT_H
#include <vision/units/vision_ros_unit_interface.h>
#include <vision/storage/homography_storage.h>
#include <vision/HomographyConfig.h>
#include <vector>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <ros/ros.h>


#define TOPVIEW_ROWS 258
#define TOPVIEW_COLS 640

#define TOPVIEW_MIN_X 0.45
#define TOPVIEW_MAX_X 1.4
#define TOPVIEW_MIN_Y -1.3
#define TOPVIEW_MAX_Y 2.3

class HomographyUnit : public VisionROSUnitInterface<vision::HomographyConfig>
{
  HomographyStorage &storage_;
  std::vector<cv::Point2f> coordinates_;
  std::vector<cv::Point2f> pixels_;
  cv::Size topview_size_;
  cv::Mat topview2world_;
  cv::Mat world2topview_;
  cv::Mat topview2cam_;
  cv::Mat world2cam_;
  std::string config_file_;

  void computeTopView();
  void loadFiles();
  void homography();

public:
  void init() override;
  void trigger() override;
  HomographyUnit(HomographyStorage &storage);
};

#endif  // VISION_UNITS_HOMOGRAPHY_UNIT_H
