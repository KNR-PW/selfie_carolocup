#include <realsense_obstacle_detector/RealSensePcl.hpp>

void RealSensePcl::pc2Callback(const sensor_msgs::PointCloud2 &msg)
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(msg, cloud);

  pcl::PointCloud<pcl::PointXYZHSV> cloud_hsv;
  convertPCXYZRGBtoXYZHSV(cloud, cloud_hsv);

  pcl::PointCloud<pcl::PointXYZHSV> cloud_filtered;
  filterColorFromPC(cloud_hsv, cloud_filtered);

  pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered_rgb;
  convertPCXYZHSVtoXYZRGB(cloud_filtered, cloud_filtered_rgb);

  pcl::PointCloud<pcl::PointXYZ> cloud_filtered_rgb_xyz;
  filterFloorFromPC(cloud_filtered_rgb, cloud_filtered_rgb_xyz);

  pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered_rgb_ready;
  filterAddColor(cloud_filtered_rgb_xyz, cloud_filtered_rgb, cloud_filtered_rgb_ready);

  sensor_msgs::PointCloud2 output_pc;
  pcl::toROSMsg(cloud_filtered_rgb_ready, output_pc);
  output_pc.header.frame_id = "camera_depth_optical_frame";
  output_pc.header.stamp = ros::Time::now();
  pc2_color_filter_publisher.publish(output_pc);
}

void RealSensePcl::filterColorFromPC(const pcl::PointCloud<pcl::PointXYZHSV>& input_pc,
                                     pcl::PointCloud<pcl::PointXYZHSV>& output_pc)
{
  for (size_t i = 0; i < input_pc.points.size(); i++)
  {
    if (input_pc.points[i].s > saturation_ && input_pc.points[i].v < value_)
    {
      output_pc.points.push_back(input_pc.points[i]);
    }
  }
}

void RealSensePcl::filterAddColor(const pcl::PointCloud<pcl::PointXYZ>& input_pc,
                                     pcl::PointCloud<pcl::PointXYZRGB>& input_rgb_pc, 
                                     pcl::PointCloud<pcl::PointXYZRGB>& output_pc)
{
  int x = 0;
  for (size_t i = 0; i < input_pc.points.size(); i++)
  {
    if(input_pc.points[i].x == input_rgb_pc.points[i].x && input_pc.points[i].y == input_rgb_pc.points[i].y && input_pc.points[i].z == input_rgb_pc.points[i].z)
    {
        output_pc.points.push_back(input_rgb_pc.points[i]);
        x++;
    }
  }
}

void RealSensePcl::filterFloorFromPC(const pcl::PointCloud<pcl::PointXYZRGB>& input_pc,
                                     pcl::PointCloud<pcl::PointXYZ>& output_pc)
{
  float in_floor_max_angle = 0.1;
  float in_max_height = 0.02;

  pcl::PointCloud<pcl::PointXYZ> in_pc;
  pcl::PointCloud<pcl::PointXYZ> out_pc;
  pcl::copyPointCloud(input_pc, in_pc);

  pcl::PointCloud<pcl::PointXYZ>::Ptr input = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(in_pc);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);

  // boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> > output = boost::make_shared(output_pc);


  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setAxis(Eigen::Vector3f(0, 1, 0));
  seg.setEpsAngle(in_floor_max_angle);


  seg.setDistanceThreshold(in_max_height);  // floor distance
  seg.setOptimizeCoefficients(true);
  seg.setInputCloud(input);
  seg.segment(*inliers, *coefficients);
  // std::cout << "ULALAL" << std::endl;
  if (inliers->indices.size() == 0)
  {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  }

  // REMOVE THE FLOOR FROM THE CLOUD
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(input);
  extract.setIndices(inliers);
  extract.setNegative(true);  // true removes the indices, false leaves only the indices
  extract.filter(*output);

  pcl::copyPointCloud(*output, output_pc);

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
