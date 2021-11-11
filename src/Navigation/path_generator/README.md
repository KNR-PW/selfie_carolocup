# path_generator
Creates a path for the vehicle to follow based on the designated offset from the center line and the center line polynomial representation

## Usage
```
. devel/setup.bash
rosrun path_generator path_generator_node.py
```

## Topics
### Subscribed topics
- `path_offset` ([std_msgs/Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))
  - Offset from the path from the center line in y axis
- `/road_lines` ([custom_msgs/RoadLines](./../../Shared/custom_msgs/msg/RoadLines.msg))
  - Polynomial coefficients for fitted lines, starting from the constant term
### Published topics
- `/path` ([nav_msgs/Path](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))
  - The path for the vehicle to follow

## Parameters
- `~interval_x` (*float*)
  - The distance between consecutive points in the path
- `~max_distance` (*float*)
  - Maximal length of the output path
