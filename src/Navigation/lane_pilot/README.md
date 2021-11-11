# lane_pilot

lane_pilot- in active mode detects obstacles on the road, changes lane and returns after overtaking. In passive mode sends constant offset.
Current version overtakes static obstacles.

## Usage

```bash
. devel/setup.bash
rosrun selfie_avoiding_obstacles lane_pilot
```

## Topics

### Subscribed topics

- `/obstacles` ([custom_msgs/Box2DArray](./../../Shared/custom_msgs/msg/Box2DArray.msg))
  - detected obstacles
- `/road_lines` ([custom_msgs/RoadLines](./../../Shared/custom_msgs/msg/RoadLines.msg))
  - contains polynomial coefficients for fitted lines
- `selfie_out/motion` ([custom_msgs/Motion](./../../Shared/custom_msgs/msg/Motion.msg))
  - distance covered by car
  
### Published topics

- `/path_offset` ([std_msgs/Float32](https://docs.ros.org/api/std_msgs/html/msg/Float32.html))
  - describes distance to middle lane, changing this value can change lane to left
- `/max_speed` ([std_msgs/Float64](https://docs.ros.org/api/std_msgs/html/msg/Float64.html))
  - current speed of car
  - describes lane, changing this value changes lane
- `/selfie_in/indicators` ([custom_msgs/Indicators](./../../Shared/custom_msgs/msg/Indicators.msg))
  - used for turning on and of turn indicators
- `/visualization/avoiding_obstacles` ([visualization_msgs/Marker](https://docs.ros.org/api/visualization_msgs/html/msg/Marker.html))
  - (only when parameter `visualization=true` visualizes found found and places in rviz)

### Used Services

- `/avoiding_obst_set_active` ([std_srvs/Empty](https://docs.ros.org/api/std_srvs/html/srv/Empty.html))
- `/avoiding_obst_set_passive` ([std_srvs/Empty](https://docs.ros.org/api/std_srvs/html/srv/Empty.html))
  - they are used to switch modes on active and passive
- `/resetLaneControl` ([std_srvs/Empty](https://docs.ros.org/api/std_srvs/html/srv/Empty.html))
  - used to reset node

## Parameters

- `visualization` (*bool*, default: true)
  - Whether or not visualization topics are active
- `max_length_of_obstacle` (*float*)
  - How long can be approached obstacle (described in regulations)
- `max_distance_to_obstacle` (*float*)
  - if obstacle in front of car is nearer than this value then car will start overtaking
- `ROI_min_x`,`ROI_min_y`,`ROI_max_x`,`ROI_max_x` (*float*)
  - describing area of interest
- `right_obst_area_min_x`,`right_obst_area_max_x`,`right_obst_area_min_y`,`right_obst_area_max_y` (*float*)
  - describing area of interest used while overtaking (checking if there is still obstacle on thhe right side of car)
- `right_lane_offset`,`left_lane_offset` (*float*, default: -0.2,0.2)
  - How far from middle of road is middle of right and left lane
- `maximum_speed` (*float*)
- `slowdown_speed` (*float*)
  - Speed used before lane is being changed
- `lane_change_speed_` (*float*)
  - Speed used when lane is being changed
- `safety_margin` (*float*)
- safety margin considering inaccurations in measuring distance, used to calculate
- `num_proof_to_slowdown` (*int*)
  - how many times in a row car should discover obstacle to start intersecting (it is used to avoid overtaking caused by static)
- `num_corners_to_detect` (*int*)
  - how many corners of box should be in area of interest to consider it as obstacle to avoid
- `lane_change_distance` (*float*)
  - how many meters it should take to change lane
