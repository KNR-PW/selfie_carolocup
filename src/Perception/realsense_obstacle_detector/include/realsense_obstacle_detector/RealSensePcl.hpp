#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <dynamic_reconfigure/server.h>
#include <realsense_obstacle_detector/RealsenseObstacleDetectorConfig.h>


class RealSensePcl
{
private:
  float hue_;
  float saturation_;
  float value_;
  float hue_angle_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber pc2_subscriber_;
  ros::Publisher pc2_color_filter_publisher;

  void pc2Callback(const sensor_msgs::PointCloud2 &msg);
  void filterColorFromPC(const pcl::PointCloud<pcl::PointXYZHSV>& input_pc,
                         pcl::PointCloud<pcl::PointXYZHSV>& output_pc);
  void convertPCXYZRGBtoXYZHSV(const pcl::PointCloud<pcl::PointXYZRGB>& input_pc,
                               pcl::PointCloud<pcl::PointXYZHSV>& output_pc);
  void convertPCXYZHSVtoXYZRGB(const pcl::PointCloud<pcl::PointXYZHSV>& input_pc,
                               pcl::PointCloud<pcl::PointXYZRGB>& output_pc);

  void dynamicReconfigureCb(realsense_obstacle_detector::RealsenseObstacleDetectorConfig& config, uint32_t level);
  dynamic_reconfigure::Server<realsense_obstacle_detector::RealsenseObstacleDetectorConfig> dr_server_;
  dynamic_reconfigure::Server<realsense_obstacle_detector::RealsenseObstacleDetectorConfig>::CallbackType dr_server_cb_;

public:
  RealSensePcl(const ros::NodeHandle& node_handle, const ros::NodeHandle& private_node_handle);
  virtual ~RealSensePcl() = default;
};