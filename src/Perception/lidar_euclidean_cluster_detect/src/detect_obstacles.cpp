#include "prepare_data.h"
#include "merging_and_clustering.h"

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

#include "gencolors.cpp"

#include "cluster.h"

#define __APP_NAME__ "euclidean_clustering"

using namespace cv;

ros::Publisher _test_cloud;

ros::Publisher _pub_cluster_cloud;
ros::Publisher _pub_cluster_cloud_box;
ros::Publisher _pub_ground_cloud;
ros::Publisher _centroid_pub;

ros::Publisher _pub_clusters_message;

ros::Publisher _pub_clusters_box_message;

ros::Publisher _pub_clusters_signs_cloud;

ros::Publisher _pub_points_lanes_cloud;

ros::Publisher _pub_detected_objects;

std_msgs::Header _velodyne_header;

std::string _output_frame;

static bool _velodyne_transform_available;
static bool _downsample_cloud;
static bool _pose_estimation;
static double _leaf_size;
static int _cluster_size_min;
static int _cluster_size_max;
static const double _initial_quat_w = 1.0;

static bool _remove_ground;  

static bool _using_sensor_cloud;
static bool _use_diffnormals;

static double _clip_min_height;
static double _clip_max_height;

static bool _keep_lanes;
static double _keep_lane_left_distance;
static double _keep_lane_right_distance;

static double _remove_points_upto;
static double _cluster_merge_threshold;
static double _clustering_distance;

static bool _use_gpu;
static std::chrono::system_clock::time_point _start, _end;

std::vector<std::vector<geometry_msgs::Point>> _way_area_points;
std::vector<cv::Scalar> _colors;
pcl::PointCloud<pcl::PointXYZ> _sensor_cloud;

static bool _use_multiple_thres;
std::vector<double> _clustering_distances;
std::vector<double> _clustering_ranges;

tf::StampedTransform *_transform;
tf::StampedTransform *_velodyne_output_transform;
tf::TransformListener *_transform_listener;
tf::TransformListener *_vectormap_transform_listener;

void publishCentroids(const ros::Publisher *in_publisher, const autoware_msgs::Centroids &in_centroids,
                      const std::string &in_target_frame, const std_msgs::Header &in_header)
{
  if (in_target_frame != in_header.frame_id)
  {
    autoware_msgs::Centroids centroids_transformed;
    centroids_transformed.header = in_header;
    centroids_transformed.header.frame_id = in_target_frame;
    for (auto i = centroids_transformed.points.begin(); i != centroids_transformed.points.end(); i++)
    {
      geometry_msgs::PointStamped centroid_in, centroid_out;
      centroid_in.header = in_header;
      centroid_in.point = *i;
      try
      {
        _transform_listener->transformPoint(in_target_frame, ros::Time(), centroid_in, in_header.frame_id,
                                            centroid_out);

        centroids_transformed.points.push_back(centroid_out.point);
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("publishCentroids: %s", ex.what());
      }
    }
    in_publisher->publish(centroids_transformed);
  } else
  {
    in_publisher->publish(in_centroids);
  }
}

void publishDetectedObjects(const autoware_msgs::CloudClusterArray &in_clusters)
{
  autoware_msgs::DetectedObjectArray detected_objects;
  detected_objects.header = in_clusters.header;

  for (size_t i = 0; i < in_clusters.clusters.size(); i++)
  {
    autoware_msgs::DetectedObject detected_object;
    detected_object.header = in_clusters.header;
    detected_object.label = "unknown";
    detected_object.score = 1.;
    detected_object.space_frame = in_clusters.header.frame_id;
    detected_object.pose = in_clusters.clusters[i].bounding_box.pose;
    detected_object.dimensions = in_clusters.clusters[i].dimensions;
    detected_object.pointcloud = in_clusters.clusters[i].cloud;
    detected_object.convex_hull = in_clusters.clusters[i].convex_hull;
    detected_object.valid = true;

    detected_objects.objects.push_back(detected_object);
  }
  _pub_detected_objects.publish(detected_objects);
}

void publishCloudClusters(const ros::Publisher* in_publisher, const autoware_msgs::CloudClusterArray& in_clusters,
                          const std::string& in_target_frame, const std_msgs::Header& in_header)
{
  if (in_target_frame != in_header.frame_id)
  {
    autoware_msgs::CloudClusterArray clusters_transformed;
    clusters_transformed.header = in_header;
    clusters_transformed.header.frame_id = in_target_frame;
    geometry_msgs::PointStamped new_point_stamped;
    geometry_msgs::PointStamped new_point_stamped_tfed;

    for (const auto& cluster : in_clusters.clusters)
    {
      autoware_msgs::CloudCluster cluster_transformed;
      cluster_transformed.header = in_header;
      try
      {
        _transform_listener->lookupTransform(in_target_frame, _velodyne_header.frame_id, ros::Time(), *_transform);
        pcl_ros::transformPointCloud(in_target_frame, *_transform, cluster.cloud, cluster_transformed.cloud);
        _transform_listener->transformPoint(in_target_frame, ros::Time(), cluster.min_point, in_header.frame_id,
                                            cluster_transformed.min_point);
        _transform_listener->transformPoint(in_target_frame, ros::Time(), cluster.max_point, in_header.frame_id,
                                            cluster_transformed.max_point);
        _transform_listener->transformPoint(in_target_frame, ros::Time(), cluster.avg_point, in_header.frame_id,
                                            cluster_transformed.avg_point);
        _transform_listener->transformPoint(in_target_frame, ros::Time(), cluster.centroid_point, in_header.frame_id,
                                            cluster_transformed.centroid_point);

        cluster_transformed.convex_hull.polygon.points.resize(cluster.convex_hull.polygon.points.size());
        cluster_transformed.convex_hull.header.frame_id = in_target_frame;
        for (size_t i = 0; i < cluster.convex_hull.polygon.points.size(); i++)
        {
          geometry_msgs::Point32 new_point32;
          new_point_stamped.header.frame_id = in_header.frame_id;
          new_point_stamped.point.x = static_cast<double>(cluster.convex_hull.polygon.points[i].x);
          new_point_stamped.point.y = static_cast<double>(cluster.convex_hull.polygon.points[i].y);
          new_point_stamped.point.z = static_cast<double>(cluster.convex_hull.polygon.points[i].z);
          _transform_listener->transformPoint(in_target_frame, ros::Time(), new_point_stamped, in_header.frame_id,
                                              new_point_stamped_tfed);
          new_point32.x = static_cast<float>(new_point_stamped_tfed.point.x);
          new_point32.y = static_cast<float>(new_point_stamped_tfed.point.y);
          new_point32.z = static_cast<float>(new_point_stamped_tfed.point.z);
          cluster_transformed.convex_hull.polygon.points[i] = new_point32;
        }

        // BoundingBox
        geometry_msgs::PoseStamped bb_pose_stamped;
        geometry_msgs::PoseStamped bb_pose_stamped_tfed;
        bb_pose_stamped.header = in_header;
        bb_pose_stamped.pose = cluster.bounding_box.pose;
        _transform_listener->transformPose(in_target_frame, bb_pose_stamped, bb_pose_stamped_tfed);
        cluster_transformed.bounding_box.pose = bb_pose_stamped_tfed.pose;

        cluster_transformed.dimensions = cluster.dimensions;
        cluster_transformed.eigen_values = cluster.eigen_values;
        cluster_transformed.eigen_vectors = cluster.eigen_vectors;

        if (_pose_estimation)
        {
          cluster_transformed.bounding_box.pose.orientation = cluster.bounding_box.pose.orientation;
        }
        else
        {
          cluster_transformed.bounding_box.pose.orientation.w = _initial_quat_w;
        }
        clusters_transformed.clusters.push_back(cluster_transformed);
      }
      catch (tf::TransformException& ex)
      {
        ROS_ERROR("publishCloudClusters: %s", ex.what());
      }
    }
    in_publisher->publish(clusters_transformed);
    publishDetectedObjects(clusters_transformed);
  }
  else
  {
    in_publisher->publish(in_clusters);
    publishDetectedObjects(in_clusters);
  }
}

void publishCloud(const ros::Publisher *in_publisher, const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header = _velodyne_header;
  in_publisher->publish(cloud_msg);
}

void publishColorCloud(const ros::Publisher *in_publisher,
                       const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header = _velodyne_header;
  in_publisher->publish(cloud_msg);
}

void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud)
{

  if (!_using_sensor_cloud)
  {
    _using_sensor_cloud = true;

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removed_points_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlanes_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr nofloor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr onlyfloor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr diffnormals_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clustered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clustered_signs_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    autoware_msgs::Centroids centroids;
    autoware_msgs::CloudClusterArray cloud_clusters;

    custom_msgs::Box3DArray box3D_cloud_clusters;

    Prepare_data* prepare_data = new Prepare_data();
    Merging_and_clustering* merging_and_clustering = 
      new Merging_and_clustering(_velodyne_header, _pose_estimation, _cluster_size_min,
                                _cluster_size_max, _cluster_merge_threshold, _clustering_distance,
                                _colors, _use_multiple_thres, _clustering_distances, 
                                _clustering_ranges);

    pcl::fromROSMsg(*in_sensor_cloud, *current_sensor_cloud_ptr);

    _velodyne_header = in_sensor_cloud->header;

    if (_remove_points_upto > 0.0)
    {
      prepare_data->removePointsFurther(current_sensor_cloud_ptr, removed_points_cloud_ptr, _remove_points_upto);
    }
    else
    {
      removed_points_cloud_ptr = current_sensor_cloud_ptr;
    }

    if (_downsample_cloud)
      prepare_data->downsampleCloud(removed_points_cloud_ptr, downsampled_cloud_ptr, _leaf_size);
    else
      downsampled_cloud_ptr = removed_points_cloud_ptr;

    prepare_data->clipCloud(downsampled_cloud_ptr, clipped_cloud_ptr, _clip_min_height, _clip_max_height);

    if (_keep_lanes)
      prepare_data->keepLanePoints(clipped_cloud_ptr, inlanes_cloud_ptr, _keep_lane_left_distance, _keep_lane_right_distance);
    else
      inlanes_cloud_ptr = clipped_cloud_ptr;

    if (_remove_ground)
    {
      prepare_data->removeFloor(inlanes_cloud_ptr, nofloor_cloud_ptr, onlyfloor_cloud_ptr, 0.02, 0.1);
      publishCloud(&_pub_ground_cloud, onlyfloor_cloud_ptr);
    }
    else
    {
      nofloor_cloud_ptr = inlanes_cloud_ptr;
    }

    publishCloud(&_pub_points_lanes_cloud, nofloor_cloud_ptr);


    diffnormals_cloud_ptr = nofloor_cloud_ptr;

    merging_and_clustering->segmentByDistance(diffnormals_cloud_ptr, colored_clustered_cloud_ptr, 
                                            colored_clustered_signs_cloud_ptr,  centroids,
                                            cloud_clusters, box3D_cloud_clusters);

    _pub_clusters_box_message.publish(box3D_cloud_clusters);

    publishColorCloud(&_pub_clusters_signs_cloud, colored_clustered_signs_cloud_ptr);
    publishColorCloud(&_pub_cluster_cloud, colored_clustered_cloud_ptr);

    if(box3D_cloud_clusters.boxes.size() > 0){
      _pub_cluster_cloud_box.publish(box3D_cloud_clusters.boxes.at(0).cloud);
    }

    centroids.header = _velodyne_header;

    publishCentroids(&_centroid_pub, centroids, _output_frame, _velodyne_header);

    cloud_clusters.header = _velodyne_header;

    publishCloudClusters(&_pub_clusters_message, cloud_clusters, _output_frame, _velodyne_header);

    _using_sensor_cloud = false;
  }
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "euclidean_cluster");

  ros::NodeHandle h;
  ros::NodeHandle private_nh("~");

  tf::StampedTransform transform;
  tf::TransformListener listener;
  tf::TransformListener vectormap_tf_listener;

  _vectormap_transform_listener = &vectormap_tf_listener;
  _transform = &transform;
  _transform_listener = &listener;

  generateColors(_colors, 255);
  
  _pub_cluster_cloud = h.advertise<sensor_msgs::PointCloud2>("/points_cluster", 1);
  _pub_cluster_cloud_box = h.advertise<sensor_msgs::PointCloud2>("/points_cluster_box", 1);
  _pub_ground_cloud = h.advertise<sensor_msgs::PointCloud2>("/points_ground", 1);
  _pub_clusters_signs_cloud = h.advertise<sensor_msgs::PointCloud2>("/points_signs", 1);
  _centroid_pub = h.advertise<autoware_msgs::Centroids>("/cluster_centroids", 1);

  _pub_points_lanes_cloud = h.advertise<sensor_msgs::PointCloud2>("/points_lanes", 1);
  _pub_clusters_message = h.advertise<autoware_msgs::CloudClusterArray>("/detection/lidar_detector/cloud_clusters", 1);

  _pub_clusters_box_message = h.advertise<custom_msgs::Box3DArray>("/obstacles", 1);

  _pub_detected_objects = h.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects", 1);

  std::string points_topic, gridmap_topic;

  _using_sensor_cloud = false;

  if (private_nh.getParam("points_node", points_topic))
  {
    ROS_INFO("euclidean_cluster > Setting points node to %s", points_topic.c_str());
  }
  else
  {
    points_topic = "/camera/depth/color/points";
  }

  _use_diffnormals = false;
  if (private_nh.getParam("use_diffnormals", _use_diffnormals))
  {
    if (_use_diffnormals)
      ROS_INFO("Euclidean Clustering: Applying difference of normals on clustering pipeline");
    else
      ROS_INFO("Euclidean Clustering: Difference of Normals will not be used.");
  }

  /* Initialize tuning parameter */
  private_nh.param("downsample_cloud", _downsample_cloud, false);
  ROS_INFO("[%s] downsample_cloud: %d", __APP_NAME__, _downsample_cloud);
  private_nh.param("remove_ground", _remove_ground, true);
  ROS_INFO("[%s] remove_ground: %d", __APP_NAME__, _remove_ground);
  private_nh.param("leaf_size", _leaf_size, 0.1);
  ROS_INFO("[%s] leaf_size: %f", __APP_NAME__, _leaf_size);
  private_nh.param("cluster_size_min", _cluster_size_min, 20);
  ROS_INFO("[%s] cluster_size_min %d", __APP_NAME__, _cluster_size_min);
  private_nh.param("cluster_size_max", _cluster_size_max, 100000);
  ROS_INFO("[%s] cluster_size_max: %d", __APP_NAME__, _cluster_size_max);
  private_nh.param("pose_estimation", _pose_estimation, false);
  ROS_INFO("[%s] pose_estimation: %d", __APP_NAME__, _pose_estimation);
  private_nh.param("clip_min_height", _clip_min_height, -1.3);
  ROS_INFO("[%s] clip_min_height: %f", __APP_NAME__, _clip_min_height);
  private_nh.param("clip_max_height", _clip_max_height, 0.5);
  ROS_INFO("[%s] clip_max_height: %f", __APP_NAME__, _clip_max_height);
  private_nh.param("keep_lanes", _keep_lanes, false);
  ROS_INFO("[%s] keep_lanes: %d", __APP_NAME__, _keep_lanes);
  private_nh.param("keep_lane_left_distance", _keep_lane_left_distance, 5.0);
  ROS_INFO("[%s] keep_lane_left_distance: %f", __APP_NAME__, _keep_lane_left_distance);
  private_nh.param("keep_lane_right_distance", _keep_lane_right_distance, 5.0);
  ROS_INFO("[%s] keep_lane_right_distance: %f", __APP_NAME__, _keep_lane_right_distance);
  private_nh.param("cluster_merge_threshold", _cluster_merge_threshold, 1.5);
  ROS_INFO("[%s] cluster_merge_threshold: %f", __APP_NAME__, _cluster_merge_threshold);
  private_nh.param<std::string>("output_frame", _output_frame, "velodyne");
  ROS_INFO("[%s] output_frame: %s", __APP_NAME__, _output_frame.c_str());

  private_nh.param("remove_points_upto", _remove_points_upto, 0.0);
  ROS_INFO("[%s] remove_points_upto: %f", __APP_NAME__, _remove_points_upto);

  private_nh.param("clustering_distance", _clustering_distance, 0.75);
  ROS_INFO("[%s] clustering_distance: %f", __APP_NAME__, _clustering_distance);

  private_nh.param("use_gpu", _use_gpu, false);
  ROS_INFO("[%s] use_gpu: %d", __APP_NAME__, _use_gpu);

  private_nh.param("use_multiple_thres", _use_multiple_thres, true);
  ROS_INFO("[%s] use_multiple_thres: %d", __APP_NAME__, _use_multiple_thres);

  std::string str_distances;
  std::string str_ranges;
  private_nh.param("clustering_distances", str_distances, std::string("[0.5,1.1,1.6,2.1,2.6]"));
  ROS_INFO("[%s] clustering_distances: %s", __APP_NAME__, str_distances.c_str());
  private_nh.param("clustering_ranges", str_ranges, std::string("[15,30,45,60]"));
    ROS_INFO("[%s] clustering_ranges: %s", __APP_NAME__, str_ranges.c_str());

  if (_use_multiple_thres)
  {
    YAML::Node distances = YAML::Load(str_distances);
    YAML::Node ranges = YAML::Load(str_ranges);
    size_t distances_size = distances.size();
    size_t ranges_size = ranges.size();
    if (distances_size == 0 || ranges_size == 0)
    {
      ROS_ERROR("Invalid size of clustering_ranges or/and clustering_distance. \
    The size of clustering distance and clustering_ranges should not be 0");
      ros::shutdown();
    }
    if ((distances_size - ranges_size) != 1)
    {
      ROS_ERROR("Invalid size of clustering_ranges or/and clustering_distance. \
    Expecting that (distances_size - ranges_size) == 1 ");
      ros::shutdown();
    }
    for (size_t i_distance = 0; i_distance < distances_size; i_distance++)
    {
      _clustering_distances.push_back(distances[i_distance].as<double>());
    }
    for (size_t i_range = 0; i_range < ranges_size; i_range++)
    {
      _clustering_ranges.push_back(ranges[i_range].as<double>());
    }
  }

  _velodyne_transform_available = false;

  ros::Subscriber sub = h.subscribe(points_topic, 1, velodyne_callback);

  ros::spin();
}


