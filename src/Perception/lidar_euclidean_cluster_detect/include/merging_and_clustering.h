
#ifndef MERGING_AND_CLUSTERING_H_
#define MERGING_AND_CLUSTERING_H_
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

class Merging_and_clustering
{
    std_msgs::Header _velodyne_header;

    bool _pose_estimation;
    int _cluster_size_min;
    int _cluster_size_max;

    double _cluster_merge_threshold;
    double _clustering_distance;

    std::vector<cv::Scalar> _colors;

    bool _use_multiple_thres;
    std::vector<double> _clustering_distances;
    std::vector<double> _clustering_ranges;


    std::vector<ClusterPtr> clusterAndColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                                        autoware_msgs::Centroids &in_out_centroids,
                                        double in_max_cluster_distance);

    void checkClusterMerge(size_t in_cluster_id, std::vector<ClusterPtr> &in_clusters,
                       std::vector<bool> &in_out_visited_clusters, std::vector<size_t> &out_merge_indices,
                       double in_merge_threshold);

    void mergeClusters(const std::vector<ClusterPtr> &in_clusters, std::vector<ClusterPtr> &out_clusters,
                   std::vector<size_t> in_merge_indices, const size_t &current_index,
                   std::vector<bool> &in_out_merged_clusters);

    void checkAllForMerge(std::vector<ClusterPtr> &in_clusters, std::vector<ClusterPtr> &out_clusters,
                      float in_merge_threshold);

    public:

        Merging_and_clustering(std_msgs::Header velodyne_header, bool pose_estimation,
                            int cluster_size_min, int cluster_size_max, double cluster_merge_threshold,
                            double clustering_distance, std::vector<cv::Scalar> colors,
                            bool use_multiple_thres, std::vector<double> clustering_distances,
                            std::vector<double> clustering_ranges) :
                            _velodyne_header(velodyne_header),
                            _pose_estimation(pose_estimation),
                            _cluster_size_min(cluster_size_min),
                            _cluster_size_max(cluster_size_max),
                            _cluster_merge_threshold(cluster_merge_threshold),
                            _clustering_distance(clustering_distance),
                            _colors(colors),
                            _use_multiple_thres(use_multiple_thres),
                            _clustering_distances(clustering_distances),
                            _clustering_ranges(clustering_ranges)
        {}

        void segmentByDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_signs_cloud_ptr,
                        autoware_msgs::Centroids &in_out_centroids, autoware_msgs::CloudClusterArray &in_out_clusters,
                        custom_msgs::Box3DArray &box3D_cloud_clusters);
};

#endif