
// int main(int argc, char **argv)
// {
//   // Initialize ROS
//   ros::init(argc, argv, "euclidean_cluster");

//   ros::NodeHandle h;
//   ros::NodeHandle private_nh("~");

//   tf::StampedTransform transform;
//   tf::TransformListener listener;
//   tf::TransformListener vectormap_tf_listener;

//   _vectormap_transform_listener = &vectormap_tf_listener;
//   _transform = &transform;
//   _transform_listener = &listener;

//   generateColors(_colors, 255);
  
//   _pub_cluster_cloud = h.advertise<sensor_msgs::PointCloud2>("/points_cluster", 1);
//   _pub_cluster_cloud_box = h.advertise<sensor_msgs::PointCloud2>("/points_cluster_box", 1);
//   _pub_ground_cloud = h.advertise<sensor_msgs::PointCloud2>("/points_ground", 1);
//   _pub_clusters_signs_cloud = h.advertise<sensor_msgs::PointCloud2>("/points_signs", 1);
//   _centroid_pub = h.advertise<autoware_msgs::Centroids>("/cluster_centroids", 1);

//   _pub_points_lanes_cloud = h.advertise<sensor_msgs::PointCloud2>("/points_lanes", 1);
//   _pub_clusters_message = h.advertise<autoware_msgs::CloudClusterArray>("/detection/lidar_detector/cloud_clusters", 1);

//   _pub_clusters_box_message = h.advertise<custom_msgs::Box3DArray>("/obstacles", 1);

//   _pub_detected_objects = h.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects", 1);

//   std::string points_topic, gridmap_topic;

//   _using_sensor_cloud = false;

//   if (private_nh.getParam("points_node", points_topic))
//   {
//     ROS_INFO("euclidean_cluster > Setting points node to %s", points_topic.c_str());
//   }
//   else
//   {
//     ROS_INFO("euclidean_cluster > No points node received, defaulting to points_raw, you can use "
//                "_points_node:=YOUR_TOPIC");
//     // points_topic = "/points_raw";
//     points_topic = "/camera/depth/color/points";

    
//   }

//   _use_diffnormals = false;
//   if (private_nh.getParam("use_diffnormals", _use_diffnormals))
//   {
//     if (_use_diffnormals)
//       ROS_INFO("Euclidean Clustering: Applying difference of normals on clustering pipeline");
//     else
//       ROS_INFO("Euclidean Clustering: Difference of Normals will not be used.");
//   }

//   /* Initialize tuning parameter */
//   private_nh.param("downsample_cloud", _downsample_cloud, false);
//   ROS_INFO("[%s] downsample_cloud: %d", __APP_NAME__, _downsample_cloud);
//   private_nh.param("remove_ground", _remove_ground, true);
//   ROS_INFO("[%s] remove_ground: %d", __APP_NAME__, _remove_ground);
//   private_nh.param("leaf_size", _leaf_size, 0.1);
//   ROS_INFO("[%s] leaf_size: %f", __APP_NAME__, _leaf_size);
//   private_nh.param("cluster_size_min", _cluster_size_min, 20);
//   ROS_INFO("[%s] cluster_size_min %d", __APP_NAME__, _cluster_size_min);
//   private_nh.param("cluster_size_max", _cluster_size_max, 100000);
//   ROS_INFO("[%s] cluster_size_max: %d", __APP_NAME__, _cluster_size_max);
//   private_nh.param("pose_estimation", _pose_estimation, false);
//   ROS_INFO("[%s] pose_estimation: %d", __APP_NAME__, _pose_estimation);
//   private_nh.param("clip_min_height", _clip_min_height, -1.3);
//   ROS_INFO("[%s] clip_min_height: %f", __APP_NAME__, _clip_min_height);
//   private_nh.param("clip_max_height", _clip_max_height, 0.5);
//   ROS_INFO("[%s] clip_max_height: %f", __APP_NAME__, _clip_max_height);
//   private_nh.param("keep_lanes", _keep_lanes, false);
//   ROS_INFO("[%s] keep_lanes: %d", __APP_NAME__, _keep_lanes);
//   private_nh.param("keep_lane_left_distance", _keep_lane_left_distance, 5.0);
//   ROS_INFO("[%s] keep_lane_left_distance: %f", __APP_NAME__, _keep_lane_left_distance);
//   private_nh.param("keep_lane_right_distance", _keep_lane_right_distance, 5.0);
//   ROS_INFO("[%s] keep_lane_right_distance: %f", __APP_NAME__, _keep_lane_right_distance);
//   private_nh.param("cluster_merge_threshold", _cluster_merge_threshold, 1.5);
//   ROS_INFO("[%s] cluster_merge_threshold: %f", __APP_NAME__, _cluster_merge_threshold);
//   private_nh.param<std::string>("output_frame", _output_frame, "velodyne");
//   ROS_INFO("[%s] output_frame: %s", __APP_NAME__, _output_frame.c_str());

//   private_nh.param("remove_points_upto", _remove_points_upto, 0.0);
//   ROS_INFO("[%s] remove_points_upto: %f", __APP_NAME__, _remove_points_upto);

//   private_nh.param("clustering_distance", _clustering_distance, 0.75);
//   ROS_INFO("[%s] clustering_distance: %f", __APP_NAME__, _clustering_distance);

//   private_nh.param("use_gpu", _use_gpu, false);
//   ROS_INFO("[%s] use_gpu: %d", __APP_NAME__, _use_gpu);

//   private_nh.param("use_multiple_thres", _use_multiple_thres, true);
//   ROS_INFO("[%s] use_multiple_thres: %d", __APP_NAME__, _use_multiple_thres);

//   std::string str_distances;
//   std::string str_ranges;
//   private_nh.param("clustering_distances", str_distances, std::string("[0.5,1.1,1.6,2.1,2.6]"));
//   ROS_INFO("[%s] clustering_distances: %s", __APP_NAME__, str_distances.c_str());
//   private_nh.param("clustering_ranges", str_ranges, std::string("[15,30,45,60]"));
//     ROS_INFO("[%s] clustering_ranges: %s", __APP_NAME__, str_ranges.c_str());

//   if (_use_multiple_thres)
//   {
//     YAML::Node distances = YAML::Load(str_distances);
//     YAML::Node ranges = YAML::Load(str_ranges);
//     size_t distances_size = distances.size();
//     size_t ranges_size = ranges.size();
//     if (distances_size == 0 || ranges_size == 0)
//     {
//       ROS_ERROR("Invalid size of clustering_ranges or/and clustering_distance. \
//     The size of clustering distance and clustering_ranges should not be 0");
//       ros::shutdown();
//     }
//     if ((distances_size - ranges_size) != 1)
//     {
//       ROS_ERROR("Invalid size of clustering_ranges or/and clustering_distance. \
//     Expecting that (distances_size - ranges_size) == 1 ");
//       ros::shutdown();
//     }
//     for (size_t i_distance = 0; i_distance < distances_size; i_distance++)
//     {
//       _clustering_distances.push_back(distances[i_distance].as<double>());
//     }
//     for (size_t i_range = 0; i_range < ranges_size; i_range++)
//     {
//       _clustering_ranges.push_back(ranges[i_range].as<double>());
//     }
//   }

//   _velodyne_transform_available = false;

//   // Create a ROS subscriber for the input point cloud
//   ros::Subscriber sub = h.subscribe(points_topic, 1, velodyne_callback);

//   // Spin
//   ros::spin();
// }
