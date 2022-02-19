#ifndef CLUSTER_H_
#define CLUSTER_H_
#pragma once

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/common/common.h>
#include <pcl/common/pca.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <pcl/segmentation/extract_clusters.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_rviz_plugins/PictogramArray.h>

#include "autoware_msgs/CloudCluster.h"
#include "custom_msgs/Box3D.h"

#include "custom_msgs/Box2D.h"
#include "custom_msgs/Box2DArray.h"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <limits>
#include <cmath>
#include <chrono>

class Cluster
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_;
  pcl::PointXYZ min_point_;
  pcl::PointXYZ max_point_;
  pcl::PointXYZ average_point_;
  pcl::PointXYZ centroid_;
  double orientation_angle_;
  float length_, width_, height_;

  jsk_recognition_msgs::BoundingBox bounding_box_;
  geometry_msgs::PolygonStamped polygon_;

  std::string label_;
  int id_;
  int r_, g_, b_;

  Eigen::Matrix3f eigen_vectors_;
  Eigen::Vector3f eigen_values_;

  bool valid_cluster_;

public:
  void SetCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_origin_cloud_ptr,
                const std::vector<int>& in_cluster_indices, std_msgs::Header in_ros_header, int in_id, int in_r,
                int in_g, int in_b, std::string in_label, bool in_estimate_pose);

  void ToROSMessage(std_msgs::Header in_ros_header, autoware_msgs::CloudCluster& out_cluster_message);

  void BoxToROSMessage(std_msgs::Header in_ros_header, custom_msgs::Box2D& out_cluster_message);

  Cluster();
  virtual ~Cluster();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetCloud();

  pcl::PointXYZ GetMinPoint();

  pcl::PointXYZ GetMaxPoint();

  pcl::PointXYZ GetAveragePoint();

  pcl::PointXYZ GetCentroid();

  jsk_recognition_msgs::BoundingBox GetBoundingBox();

  geometry_msgs::PolygonStamped GetPolygon();

  double GetOrientationAngle();

  float GetLength();

  float GetWidth();

  float GetHeight();

  int GetId();

  std::string GetLabel();

  Eigen::Matrix3f GetEigenVectors();

  Eigen::Vector3f GetEigenValues();

  bool IsValid();

  void SetValidity(bool in_valid);

  bool IsSign();

  pcl::PointCloud<pcl::PointXYZ>::Ptr JoinCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr);

};

typedef boost::shared_ptr<Cluster> ClusterPtr;

#endif /* CLUSTER_H_ */
