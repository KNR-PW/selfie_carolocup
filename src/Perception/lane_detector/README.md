# Detect markings
![markings_pic](https://user-images.githubusercontent.com/28540485/54884320-bb2aee00-4e6f-11e9-8b67-3f326029e4e9.png)
## Usage
```
. devel/setup.bash
rosrun lane_detector lane_detector
```
### Launch files:
- `hom_cut_maker.launch`
  - Create mask of homography black area cover.
- `lane_detector_example.launch`
  - Default.
- `tune_params.launch`
  - Activate slider for changing vision params.
## Topics
### Subscribed topics
- `camera_basler/image_rect` ([sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html))
  - Image from camera
### Published topics
- `/road_lines` ([custom_msgs/RoadLines](./../../Shared/custom_msgs/msg/RoadLines.msg))
  - Polynomial coefficients for fitted lines, starting from the constant term
- `/intersection/stop` ([custom_msgs/IntersectionStop](./../../Shared/custom_msgs/msg/IntersectionStop.msg))
  - Distance in meters to intersection
- `/starting_line` ([std_msgs/Float32](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32.html))
  - Distance in meters to staring line

### Used Services
- `/reset_vison` ([std_srvs/Empty](https://docs.ros.org/api/std_srvs/html/srv/Empty.html))
  - used to reset vision

## Scripts
#### make_hom_cut_file.py
  - make yaml file containts mask of homography black area cover.

## Parameters
###
- `config_file` (*string*)
  - Path to yaml file with homography matrix
- `hom_cut_file` (*string*)
  - Path to yaml file with mask of homography black area cover
- `debug_mode` (*bool*)
  - Whether or not opencv visualization windows are displayed
- `real_window_size` (*float*)
  - Size of adaptive threshold window (m)
- `threshold_c` (*int*)
  - Constant subtracted from the mean or weighted mean
- `tune_params_mode` (*bool*)
  - special mode of the node to tune thresholds parameters
- `max_mid_line_distance` (*float*)
  - max side line distance to merge (used to merge middle lines)
- `max_mid_line_gap` (*float*)
  - max line distance to merge (used to merge middle lines)
- `pf_num_samples` (*int*)
  - number of particles in particle filter
- `pf_num_points_` (*int*)
  - number of control points describing polynomial line in particle filter
- `pf_std_min` (*float*)
  - min standard deviation of normal distribution using to move control points during particle filter perception (linear relationship based on distance of the car)
- `pf_std_max` (*float*)
  - max standard deviation of normal distribution using to move control points during particle filter perception (linear relationship based on distance of the car)
- `pf_num_samples_vis` (*int*)
  - number of particles in particle filter to visualize