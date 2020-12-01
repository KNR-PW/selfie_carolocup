#include <vision/units/debugger_unit.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>
#include <iostream>



DebuggerUnit::DebuggerUnit(DebuggerStorage &storage): storage_(storage)
{
}
void DebuggerUnit::init()
{
  addVis("image_rect", storage_.image_rect_);
  addVis("homography_frame", storage_.homography_frame_);
  for (auto &vis: vis_)
  {
    cv::namedWindow(vis.first);
  }

}

void DebuggerUnit::trigger()
{
  for (auto &vis: vis_)
  {
    cv::imshow(vis.first, vis.second.get());
  }
  cv::waitKey(1);
}

void DebuggerUnit::addVis(const std::string & name, cv::Mat & mat)
{
  std::reference_wrapper<cv::Mat> wrap(mat);
  std::pair<std::string, std::reference_wrapper<cv::Mat>> pair(name, wrap);
  vis_.insert(pair);
}

void DebuggerUnit::removeVis(const std::string &name)
{
  auto it = vis_.find(name);
  cv::destroyWindow(it->first);
  vis_.erase(it);
}

DebuggerUnit::~DebuggerUnit()
{
  for (auto &vis: vis_)
  {
    cv::destroyWindow(vis.first);
  }
}
