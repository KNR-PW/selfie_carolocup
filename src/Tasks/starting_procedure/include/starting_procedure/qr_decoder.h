/***Copyright ( c) 2019, KNR Selfie*
 * This code is licensed under BSD license (see LICENSE for details)
 **/
#ifndef STARTING_PROCEDURE_QR_DECODER_H
#define STARTING_PROCEDURE_QR_DECODER_H

#include <ros/ros.h>
#include <zbar.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/algorithm/cxx11/any_of.hpp>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <list>

class QrDecoder
{
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber image_sub_;
  ros::ServiceServer start_serv_;
  ros::ServiceServer stop_serv_;
  ros::Publisher gate_open_pub_;

  zbar::ImageScanner zbar_scanner_;
  zbar::Image zbar_image_;

  bool init_;

  cv::Point tr_;
  cv::Point tl_;
  cv::Point br_;
  cv::Point bl_;

  float trans_width_;
  float trans_height_;
  cv::Mat M_;

  bool visualization_;
  float min_detect_rate_;
  int iterations_to_valid_;
  float detect_rate_;

  int count_frame_;
  int count_bar_;
  int count_valid_iterations_;

  ros::Timer rate_timer_;
  cv_bridge::CvImagePtr cv_ptr;

  bool startSearching(std_srvs::Empty::Request& rq, std_srvs::Empty::Response& rp);
  bool stopSearching(std_srvs::Empty::Request& rq, std_srvs::Empty::Response& rp);

  void imageRectCallback(const sensor_msgs::Image::ConstPtr msg);
  void decodeImage(const cv_bridge::CvImagePtr img);
  void calcRate(const ros::TimerEvent& time);

public:
  QrDecoder(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
};
#endif  // STARTING_PROCEDURE_QR_DECODER_H
