#include <vision/units/homography_unit.h>

#include <std_msgs/String.h>
#include <iostream>


void HomographyUnit::computeTopView()
{
  topview_size_ = cv::Size(TOPVIEW_COLS, TOPVIEW_ROWS);

  // Choose corner points (in real-world coordinates)
  coordinates_.emplace_back(TOPVIEW_MIN_X, TOPVIEW_MIN_Y);
  coordinates_.emplace_back(TOPVIEW_MIN_X, TOPVIEW_MAX_Y);
  coordinates_.emplace_back(TOPVIEW_MAX_X, TOPVIEW_MIN_Y);
  coordinates_.emplace_back(TOPVIEW_MAX_X, TOPVIEW_MAX_Y);

  pixels_.emplace_back(topview_size_.width, topview_size_.height);
  pixels_.emplace_back(0, topview_size_.height);
  pixels_.emplace_back(topview_size_.width, 0);
  pixels_.emplace_back(0, 0);

  topview2world_ = cv::findHomography(pixels_, coordinates_);
  world2topview_ = topview2world_.inv();

  topview2cam_ = world2cam_ * topview2world_;
}

void HomographyUnit::homography()
{
  cv::warpPerspective(storage_.image_rect_, storage_.homography_frame_, topview2cam_, topview_size_,
			                      cv::INTER_CUBIC | cv::WARP_INVERSE_MAP);

}

void HomographyUnit::init()
{
  loadFiles();
  computeTopView();

}

void HomographyUnit::trigger()
{
  homography();
}

HomographyUnit::HomographyUnit(HomographyStorage &storage): storage_(storage)
{
}
void HomographyUnit::loadFiles()
{
  cv::FileStorage fs1;
  if (pnh_.getParam("config_file", config_file_) && fs1.open(config_file_, cv::FileStorage::READ))
  {
    fs1["world2cam"] >> world2cam_;
    fs1.release();
  }
  else
  {
    // default homography matrix
    ROS_INFO("Default homography matrix");
    world2cam_ =
        (cv::Mat_<double>(3, 3) << -2866.861836770729, 1637.56986272477, -421.4096359156129, -2206.631043183711,
         -117.8258687217328, -738.93303219503, -4.411887214969603, -0.1088704319653951, -0.5957339297068464);
  }

}

