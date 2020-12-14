/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef VISION_UNITS_DEBUGGER_UNIT_H
#define VISION_UNITS_DEBUGGER_UNIT_H
#include <vision/units/vision_ros_unit_interface.h>
#include <vision/storage/debugger_storage.h>
#include <vision/DebuggerConfig.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <map>
#include <functional>

class DebuggerUnit : public VisionROSUnitInterface<vision::DebuggerConfig>
{
  private:
  //std::map<std::string,std::reference_wrapper<cv::Mat>> vis_;
  //std::map<std::string,image_transport::Publisher> visp_;
  std::map<std::string,std::pair<image_transport::Publisher, std::reference_wrapper<cv::Mat>>> vis_;
  void addVis(const std::string &name, cv::Mat &mat);
  void removeVis(const std::string &name);
  image_transport::ImageTransport it_;

  public:
  DebuggerStorage storage_;
  DebuggerUnit(DebuggerStorage& storage);
  ~DebuggerUnit();
  void trigger() override;
  void init() override;
  
};

#endif  // VISION_UNITS_DEBUGGER_UNIT_H
