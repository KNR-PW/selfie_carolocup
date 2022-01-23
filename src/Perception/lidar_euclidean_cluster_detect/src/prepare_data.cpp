#include "prepare_data.h"


void Prepare_data::keepLanePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_left_lane_threshold = 1.5,
                    float in_right_lane_threshold = 1.5)
{
  pcl::PointIndices::Ptr far_indices(new pcl::PointIndices);
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
  {
    pcl::PointXYZ current_point;
    current_point.x = in_cloud_ptr->points[i].x;
    current_point.y = in_cloud_ptr->points[i].y;
    current_point.z = in_cloud_ptr->points[i].z;

    if (current_point.y > (in_left_lane_threshold) || current_point.y < -1.0 * in_right_lane_threshold)
    {
      far_indices->indices.push_back(i);
    }
  }
  out_cloud_ptr->points.clear();
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(in_cloud_ptr);
  extract.setIndices(far_indices);
  extract.setNegative(true);  // true removes the indices, false leaves only the indices
  extract.filter(*out_cloud_ptr);
}

void Prepare_data::removeFloor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr out_nofloor_cloud_ptr,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr out_onlyfloor_cloud_ptr, float in_max_height = 0.02,
                 float in_floor_max_angle = 0.1)
{

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setAxis(Eigen::Vector3f(0, 1, 0));
  seg.setEpsAngle(in_floor_max_angle);

  seg.setDistanceThreshold(in_max_height);  // floor distance
  seg.setOptimizeCoefficients(true);
  seg.setInputCloud(in_cloud_ptr);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0)
  {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  }

  // REMOVE THE FLOOR FROM THE CLOUD
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(in_cloud_ptr);
  extract.setIndices(inliers);
  extract.setNegative(true);  // true removes the indices, false leaves only the indices
  extract.filter(*out_nofloor_cloud_ptr);

  // EXTRACT THE FLOOR FROM THE CLOUD
  extract.setNegative(false);  // true removes the indices, false leaves only the indices
  extract.filter(*out_onlyfloor_cloud_ptr);
}

void Prepare_data::downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_leaf_size = 0.2)
{
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(in_cloud_ptr);
  sor.setLeafSize((float) in_leaf_size, (float) in_leaf_size, (float) in_leaf_size);
  sor.filter(*out_cloud_ptr);
}

void Prepare_data::clipCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
               pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_min_height =-1.3, float in_max_height = 0.5)
{
  out_cloud_ptr->points.clear();
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
  {
    if (in_cloud_ptr->points[i].y >= in_min_height && in_cloud_ptr->points[i].y <= in_max_height)
    {
      out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
    }
  }
}

void Prepare_data::removePointsUpTo(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, const double in_distance)
{
  out_cloud_ptr->points.clear();
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
  {
    float origin_distance = sqrt(pow(in_cloud_ptr->points[i].x, 2) + pow(in_cloud_ptr->points[i].z, 2));
    if (origin_distance > in_distance)
    {
      out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
    }
  }
}

void Prepare_data::removePointsFurther(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, const double in_distance)
{
  out_cloud_ptr->points.clear();
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
  {
    float origin_distance = sqrt(pow(in_cloud_ptr->points[i].x, 2) + pow(in_cloud_ptr->points[i].z, 2));
    if (origin_distance < in_distance && in_cloud_ptr->points[i].x < 0.6 && in_cloud_ptr->points[i].x > -0.6 )
    {
      out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
    }
  }
}