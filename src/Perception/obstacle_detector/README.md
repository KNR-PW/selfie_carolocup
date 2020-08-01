# obstacle_detector

## obstacle_detector

![boxes_resized](https://user-images.githubusercontent.com/28540485/48948585-42556d00-ef35-11e8-8e83-6f161eb9e080.png)
Detects seperate boxes based on laser scans,
connecting nearby points and approximating invisible ones.
Detected boxes are transformed to output_frame.

### Subscribed topics
- `/scan` ([sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html))
  - point cloud published by LIDAR sensor
### Published topics
- `/obstacles` (custom_msgs/PolygonArray.msg)
  - Polygon Array contains vertexes of every rectangular obstacle in local XY
- `/visualization_lines` ([visualization_msgs/Marker](http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html))
  - (optional) Visualization of lines
- `/visualization_obstacles` ([visualization_msgs/Marker](http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html))
  - (optional) Visualization of obstacles
## Parameters
###
- `debug` (*bool*, default: false)
  - Whether or not visualization markers are published
- `max range` (*float*, default: 1.0)
  - Max obstacles detection range (m)
- `min_range` (*float*, default: 0.03)
  - Min obstacles detection range (m)
- `obstacles_frame` (*string*, default: "laser")
  - The frame attached to the local map
- `visualization_frame` (*string*, default: "base_link")
  - The frame attached to the local map visualization
- `output_frame` (*string*, default: "base_link")
  - The frame output is attached to
- `lidar_offset` (*float*, default: 0.0)
  - Lidar offset (m)
- `segment_threshold` (*float*, default: 0.03)
  - Max distance difference between segments (m)
- `max_segment_size` (*float*, default: 0.5)
  - Max size of segment (m)
- `min_segment_size` (*float*, default: 0.04)
  - Min size of segment (m)
- `min_to_divide` (*float*, default: 0.03)
  - Min distance to point in segment to divide line (m)
- `upside_down_` (*bool*, default: false)
  - Whether or not lidar is upside down
