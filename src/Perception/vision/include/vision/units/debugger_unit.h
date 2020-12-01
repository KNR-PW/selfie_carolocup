/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef VISION_UNITS_DEBUGGER_UNIT_H
#define VISION_UNITS_DEBUGGER_UNIT_H
#include <vision/units/vision_ros_unit_interface.h>
#include <vision/storage/debugger_storage.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <map>
#include <functional>

class DebuggerUnit : public VisionUnitInterface
{
  private:
  std::map<std::string, std::reference_wrapper<cv::Mat>> vis_;
  void addVis(const std::string &name, cv::Mat &mat);
  void removeVis(const std::string &name);

  public:
  DebuggerStorage storage_;
  DebuggerUnit(DebuggerStorage& storage);
  ~DebuggerUnit();
  void virtual trigger() override;
  void virtual init() override;
  
};

#endif  // VISION_UNITS_DEBUGGER_UNIT_H
