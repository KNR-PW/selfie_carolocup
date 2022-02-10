#include "merging_and_clustering.h"

// geometry_msgs::Point transformPoint(const geometry_msgs::Point& point, const tf::Transform& tf)
// {
//   tf::Point tf_point;
//   tf::pointMsgToTF(point, tf_point);

//   tf_point = tf * tf_point;

//   geometry_msgs::Point ros_point;
//   tf::pointTFToMsg(tf_point, ros_point);

//   return ros_point;
// }

// TODO nie dziala, zmienione z -> y
std::vector<ClusterPtr> Merging_and_clustering::clusterAndColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                                        autoware_msgs::Centroids &in_out_centroids,
                                        double in_max_cluster_distance = 0.5)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  // create 2d pc
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*in_cloud_ptr, *cloud_2d);

  if (cloud_2d->points.size() > 0)
    tree->setInputCloud(cloud_2d);

  std::vector<pcl::PointIndices> cluster_indices;

  // perform clustering on 2d cloud
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(in_max_cluster_distance);  //in_max_cluster_distance - 0.5
  ec.setMinClusterSize(_cluster_size_min); // _cluster_size_min
  ec.setMaxClusterSize(_cluster_size_max); //_cluster_size_max (10000) - 500
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_2d);
  ec.extract(cluster_indices);
  // use indices on 3d cloud

  /////////////////////////////////
  //---  3. Color clustered points
  /////////////////////////////////
  unsigned int k = 0;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

  std::vector<ClusterPtr> clusters;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);//coord + color
  // cluster
  for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    ClusterPtr cluster(new Cluster());
    cluster->SetCloud(in_cloud_ptr, it->indices, _velodyne_header, k, (int) _colors[k].val[0],
                      (int) _colors[k].val[1],
                      (int) _colors[k].val[2], "", _pose_estimation);
    clusters.push_back(cluster);

    k++;
  }
  std::cout << "Clusters: " << k << std::endl;
  return clusters;
}

void Merging_and_clustering::checkClusterMerge(size_t in_cluster_id, std::vector<ClusterPtr> &in_clusters,
                       std::vector<bool> &in_out_visited_clusters, std::vector<size_t> &out_merge_indices,
                       double in_merge_threshold)
{
  // std::cout << "checkClusterMerge" << std::endl;
  pcl::PointXYZ point_a = in_clusters[in_cluster_id]->GetCentroid();
  for (size_t i = 0; i < in_clusters.size(); i++)
  {
    if (i != in_cluster_id && !in_out_visited_clusters[i])
    {
      pcl::PointXYZ point_b = in_clusters[i]->GetCentroid();
      double distance = sqrt(pow(point_b.x - point_a.x, 2) + pow(point_b.z - point_a.z, 2));
      if (distance <= in_merge_threshold)
      {
        in_out_visited_clusters[i] = true;
        out_merge_indices.push_back(i);
        // std::cout << "Merging " << in_cluster_id << " with " << i << " dist:" << distance << std::endl;
        checkClusterMerge(i, in_clusters, in_out_visited_clusters, out_merge_indices, in_merge_threshold);
      }
    }
  }
}

void Merging_and_clustering::mergeClusters(const std::vector<ClusterPtr> &in_clusters, std::vector<ClusterPtr> &out_clusters,
                   std::vector<size_t> in_merge_indices, const size_t &current_index,
                   std::vector<bool> &in_out_merged_clusters)
{
  // std::cout << "mergeClusters:" << in_merge_indices.size() << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB> sum_cloud;
  pcl::PointCloud<pcl::PointXYZ> mono_cloud;
  ClusterPtr merged_cluster(new Cluster());
  for (size_t i = 0; i < in_merge_indices.size(); i++)
  {
    sum_cloud += *(in_clusters[in_merge_indices[i]]->GetCloud());
    in_out_merged_clusters[in_merge_indices[i]] = true;
  }
  std::vector<int> indices(sum_cloud.points.size(), 0);
  for (size_t i = 0; i < sum_cloud.points.size(); i++)
  {
    indices[i] = i;
  }

  if (sum_cloud.points.size() > 0)
  {
    pcl::copyPointCloud(sum_cloud, mono_cloud);
    merged_cluster->SetCloud(mono_cloud.makeShared(), indices, _velodyne_header, current_index,
                             (int) _colors[current_index].val[0], (int) _colors[current_index].val[1],
                             (int) _colors[current_index].val[2], "", _pose_estimation);
    out_clusters.push_back(merged_cluster);
  }
}

void Merging_and_clustering::checkAllForMerge(std::vector<ClusterPtr> &in_clusters, std::vector<ClusterPtr> &out_clusters,
                      float in_merge_threshold)
{
  // std::cout << "checkAllForMerge" << std::endl;
  std::vector<bool> visited_clusters(in_clusters.size(), false);
  std::vector<bool> merged_clusters(in_clusters.size(), false);
  size_t current_index = 0;
  for (size_t i = 0; i < in_clusters.size(); i++)
  {
    if (!visited_clusters[i])
    {
      visited_clusters[i] = true;
      std::vector<size_t> merge_indices;
      checkClusterMerge(i, in_clusters, visited_clusters, merge_indices, in_merge_threshold);
      mergeClusters(in_clusters, out_clusters, merge_indices, current_index++, merged_clusters);
    }
  }
  for (size_t i = 0; i < in_clusters.size(); i++)
  {
    // check for clusters not merged, add them to the output
    if (!merged_clusters[i])
    {
      out_clusters.push_back(in_clusters[i]);
    }
  }

  // ClusterPtr cluster(new Cluster());
}

void Merging_and_clustering::segmentByDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_signs_cloud_ptr,
                       autoware_msgs::Centroids &in_out_centroids, autoware_msgs::CloudClusterArray &in_out_clusters,
                       custom_msgs::Box3DArray &box3D_cloud_clusters)
{
  // cluster the pointcloud according to the distance of the points using different thresholds (not only one for the
  // entire pc)
  // in this way, the points farther in the pc will also be clustered

  // 0 => 0-15m d=0.5
  // 1 => 15-30 d=1
  // 2 => 30-45 d=1.6
  // 3 => 45-60 d=2.1
  // 4 => >60   d=2.6

  std::vector<ClusterPtr> all_clusters;

  if (!_use_multiple_thres)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
    {
      pcl::PointXYZ current_point;
      current_point.x = in_cloud_ptr->points[i].x;
      current_point.y = in_cloud_ptr->points[i].y;
      current_point.z = in_cloud_ptr->points[i].z;

      cloud_ptr->points.push_back(current_point);
    }
    all_clusters = clusterAndColor(cloud_ptr, out_cloud_ptr, in_out_centroids, _clustering_distance);

} else
  {
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_segments_array(5);
    for (unsigned int i = 0; i < cloud_segments_array.size(); i++)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      cloud_segments_array[i] = tmp_cloud;
    }

    for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
    {
      pcl::PointXYZ current_point;
      current_point.x = in_cloud_ptr->points[i].x;
      current_point.y = in_cloud_ptr->points[i].y;
      current_point.z = in_cloud_ptr->points[i].z;

      float origin_distance = sqrt(pow(current_point.x, 2) + pow(current_point.z, 2));

      if (origin_distance < _clustering_ranges[0])
      {
        cloud_segments_array[0]->points.push_back(current_point);
      }
      else if (origin_distance < _clustering_ranges[1])
      {
        cloud_segments_array[1]->points.push_back(current_point);

      }else if (origin_distance < _clustering_ranges[2])
      {
        cloud_segments_array[2]->points.push_back(current_point);

      }else if (origin_distance < _clustering_ranges[3])
      {
        cloud_segments_array[3]->points.push_back(current_point);

      }else
      {
        cloud_segments_array[4]->points.push_back(current_point);
      }
    }

    std::vector<ClusterPtr> local_clusters;
    for (unsigned int i = 0; i < cloud_segments_array.size(); i++)
    {
      local_clusters = clusterAndColor(cloud_segments_array[i], out_cloud_ptr, in_out_centroids, _clustering_distances[i]);
      all_clusters.insert(all_clusters.end(), local_clusters.begin(), local_clusters.end());
    }
  }

  // Clusters can be merged or checked in here
  //....
  // check for mergable clusters
  std::vector<ClusterPtr> mid_clusters;
  std::vector<ClusterPtr> final_clusters;

  if (all_clusters.size() > 0)
    checkAllForMerge(all_clusters, mid_clusters, _cluster_merge_threshold);
  else
    mid_clusters = all_clusters;

  if (mid_clusters.size() > 0)
    checkAllForMerge(mid_clusters, final_clusters, _cluster_merge_threshold);
  else
    final_clusters = mid_clusters;

    // Get final PointCloud to be published
    for (unsigned int i = 0; i < final_clusters.size(); i++)
    {
      if (final_clusters[i]->IsValid())
      {
        *out_cloud_ptr = *out_cloud_ptr + *(final_clusters[i]->GetCloud());

        jsk_recognition_msgs::BoundingBox bounding_box = final_clusters[i]->GetBoundingBox();
        geometry_msgs::PolygonStamped polygon = final_clusters[i]->GetPolygon();
        jsk_rviz_plugins::Pictogram pictogram_cluster;
        pictogram_cluster.header = _velodyne_header;

        // PICTO
        pictogram_cluster.mode = pictogram_cluster.STRING_MODE;
        pictogram_cluster.pose.position.x = final_clusters[i]->GetMaxPoint().x;
        pictogram_cluster.pose.position.y = final_clusters[i]->GetMaxPoint().y;
        pictogram_cluster.pose.position.z = final_clusters[i]->GetMaxPoint().z;
        tf::Quaternion quat(0.0, -0.7, 0.0, 0.7);
        tf::quaternionTFToMsg(quat, pictogram_cluster.pose.orientation);
        pictogram_cluster.size = 4;
        std_msgs::ColorRGBA color;
        color.a = 1;
        color.r = 1;
        color.g = 1;
        color.b = 1;
        pictogram_cluster.color = color;
        pictogram_cluster.character = std::to_string(i);
        // PICTO

        // pcl::PointXYZ min_point = final_clusters[i]->GetMinPoint();
        // pcl::PointXYZ max_point = final_clusters[i]->GetMaxPoint();
        pcl::PointXYZ center_point = final_clusters[i]->GetCentroid();
        geometry_msgs::Point centroid;
        centroid.x = center_point.x;
        centroid.y = center_point.y;
        centroid.z = center_point.z;
        bounding_box.header = _velodyne_header;
        polygon.header = _velodyne_header;

        if (final_clusters[i]->IsValid())
        {
          in_out_centroids.points.push_back(centroid);

          autoware_msgs::CloudCluster cloud_cluster;
          custom_msgs::Box3D box3D_cloud_cluster;

          
          final_clusters[i]->ToROSMessage(_velodyne_header, cloud_cluster);
          in_out_clusters.clusters.push_back(cloud_cluster);

          final_clusters[i]->BoxToROSMessage(_velodyne_header, box3D_cloud_cluster);
          box3D_cloud_clusters.boxes.push_back(box3D_cloud_cluster);
        }

        if (final_clusters[i]->IsSign())
        {
          *out_signs_cloud_ptr = *out_signs_cloud_ptr + *(final_clusters[i]->GetCloud());
        }
      }
    }
}