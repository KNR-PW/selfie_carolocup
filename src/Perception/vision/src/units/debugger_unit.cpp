#include <vision/units/debugger_unit.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>
#include <iostream>



DebuggerUnit::DebuggerUnit(DebuggerStorage &storage):VisionROSUnitInterface<vision::DebuggerConfig>("~/debugger"), storage_(storage), it_(nh_)
{
}
void DebuggerUnit::init()
{
  addVis("/image_rectaaaaaaa", storage_.image_rect_);
  addVis("/homography_frameaaaaa", storage_.homography_frame_);
  

}

void DebuggerUnit::trigger()
{
  
  //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",vis_.at("image_rect").get() ).toImageMsg();
  //visp_.at("image_rect").publish(msg);
  for(auto & v:vis_)
  {
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"mono8" ,v.second.second ).toImageMsg();
 
    v.second.first.publish(msg);
    std::cout<<"pub"<<std::endl;


  }
}

void DebuggerUnit::addVis(const std::string & name, cv::Mat & mat)
{
  //std::reference_wrapper<cv::Mat> wrap(mat);
  //std::pair<std::string, std::reference_wrapper<cv::Mat>>  vis(name, wrap);
  //vis_.insert(vis);

  //std::pair<std::string, image_transport::Publisher>  visp(name, it_.advertise(name, 1));
  //visp_.insert(visp);
  std::reference_wrapper<cv::Mat> wrap(mat);
  std::pair<image_transport::Publisher, std::reference_wrapper<cv::Mat>>  p(it_.advertise(name, 1), wrap);
  std::pair<std::string, std::pair<image_transport::Publisher, std::reference_wrapper<cv::Mat>>> p2(name, p);
  vis_.insert(p2);

}

void DebuggerUnit::removeVis(const std::string &name)
{
}

DebuggerUnit::~DebuggerUnit()
{
}
