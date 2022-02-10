#ifndef PREPARE_DATA_H_
#define PREPARE_DATA_H_
#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <limits>
#include <cmath>

#include <ros/ros.h>

#include "custom_msgs/Box3D.h"
#include "custom_msgs/Box3DArray.h"

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

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/common/common.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <pcl/segmentation/extract_clusters.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

#include "autoware_msgs/Centroids.h"
#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

#include <tf/tf.h>

#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include <opencv2/core/version.hpp>

#include "cluster.h"


using namespace cv;

class Prepare_data
{
    public:
        void keepLanePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_left_lane_threshold,
                        float in_right_lane_threshold);

        void removeFloor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr out_nofloor_cloud_ptr,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr out_onlyfloor_cloud_ptr, float in_max_height,
                    float in_floor_max_angle);

        void downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_leaf_size);

        void clipCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_min_height, float in_max_height);

        void removePointsUpTo(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, const double in_distance);

        void removePointsFurther(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, const double in_distance);
};

#endif