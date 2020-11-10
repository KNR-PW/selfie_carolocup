#include <realsense_obstacle_detector/RealSensePcl.hpp>

void RealSensePcl::pc2Callback(const sensor_msgs::PointCloud2 &msg)
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(msg, cloud);

  pcl::PointCloud<pcl::PointXYZHSV> cloud_hsv;
  convertPCXYZRGBtoXYZHSV(cloud, cloud_hsv);

  pcl::PointCloud<pcl::PointXYZHSV> cloud_filtered;
  filterColorFromPC(cloud_hsv, cloud_filtered);

  sensor_msgs::PointCloud2 output_pc;
  pcl::toROSMsg(cloud_filtered, output_pc);
  output_pc.header.frame_id = "camera_depth_optical_frame";
  output_pc.header.stamp = ros::Time::now();
  pc2_color_filter_publisher.publish(output_pc);
}

void RealSensePcl::filterColorFromPC(const pcl::PointCloud<pcl::PointXYZHSV>& input_pc,
                                     pcl::PointCloud<pcl::PointXYZHSV>& output_pc)
{
  for (size_t i = 0; i < input_pc.points.size(); i++)
  {
    if (input_pc.points[i].s < saturation_ && input_pc.points[i].v > value_)
    {
      output_pc.points.push_back(input_pc.points[i]);
    }
  }
}

void RealSensePcl::convertPCXYZRGBtoXYZHSV(const pcl::PointCloud<pcl::PointXYZRGB>& input_pc,
                                           pcl::PointCloud<pcl::PointXYZHSV>& output_pc)
{
  output_pc.width = input_pc.width;
  output_pc.height = input_pc.height;

  for (size_t i = 0; i < input_pc.points.size(); i++)
  {
    pcl::PointXYZHSV point;
    pcl::PointXYZRGBtoXYZHSV(input_pc.points[i], point);
    output_pc.points.push_back(point);
  }
}

void RealSensePcl::convertPCXYZHSVtoXYZRGB(const pcl::PointCloud<pcl::PointXYZHSV>& input_pc,
                                           pcl::PointCloud<pcl::PointXYZRGB>& output_pc)
{
  output_pc.width = input_pc.width;
  output_pc.height = input_pc.height;

  for (size_t i = 0; i < input_pc.points.size(); i++)
  {
    pcl::PointXYZRGB point;
    pcl::PointXYZHSVtoXYZRGB(input_pc.points[i], point);
    output_pc.points.push_back(point);
  }
}

void RealSensePcl::dynamicReconfigureCb(
  realsense_obstacle_detector::RealsenseObstacleDetectorConfig& config,
  uint32_t level)
{
  hue_ = config.hue;
  hue_angle_ = config.hue_angle;
  saturation_ = config.saturation;
  value_ = config.value;
}

RealSensePcl::RealSensePcl(const ros::NodeHandle& node_handle, const ros::NodeHandle& private_node_handle)
  : nh_(node_handle)
  , pnh_(private_node_handle)
  , dr_server_cb_(boost::bind(&RealSensePcl::dynamicReconfigureCb, this, _1, _2))
{
  pnh_.param<float>("hue", hue_, 0.0);
  pnh_.param<float>("saturation", saturation_, 0.2);
  pnh_.param<float>("value", value_, 0.8);

  pc2_subscriber_ = nh_.subscribe("camera/depth/color/points",
                                  1000,
                                  &RealSensePcl::pc2Callback, this);
  pc2_color_filter_publisher = pnh_.advertise<sensor_msgs::PointCloud2>("pc2/filtered", 1);

  dr_server_.setCallback(dr_server_cb_);
}
