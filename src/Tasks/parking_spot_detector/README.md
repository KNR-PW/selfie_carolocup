# parking_spot_detector
Search action server- manages speed of car while searching for free place in parking zone
Server uses [search.action](./../../Shared/custom_msgs/action/search.action) to communicate with client
## Usage
```
. devel/setup.bash
rosrun selfie_park detect_parking_spot
```
## Topics
### Action name
- `task/parking_spot_detector`

### Subscribed topics
- `/obstacles` ([selfie_msgs/PolygonArray](https://github.com/KNR-Selfie/selfie_carolocup2020/wiki/Messages-and-actions))
  - detected obstacles
- `/selfie_out/motion` ([custom_msgs/movement](./../../Shared/custom_msgs/msg/Motion.msg))
  - from this message distance is used
 
### Published topics
- `/max_speed` ([std_msgs/Float64](https://docs.ros.org/api/std_msgs/html/msg/Float64.html))
  - current speed of car
- `/visualization/free_place` ([visualization_msgs/Marker](https://docs.ros.org/api/visualization_msgs/html/msg/Marker.html))
  - (only when parameter `visualization=true` visualizes found boxes and places in rviz)
-  `/state/task` ([std_msgs/Int8](https://docs.ros.org/api/std_msgs/html/msg/Int8.html))


## Parameters
###
 - `point_min_x`,`point_min_y`,`point_max_x`,`point_max_x` (*float*)
   - describing area of interest
 - `visualization_in_searching` (*bool*)
   - Whether or not visualization topics are active
 - `default_speed_in_parking_zone` (*float*)
 - `speed_when_found_place` (*float*)
   - speed when found possible free place
 - `box_angle_deg` (*float*)
   - describes maximum angle (in degrees) between car and found place (used mainly in filtering out wrong places)
 - `max_distance_to_free_place` (*float*)
   - describes maximum angle between car and found place (used mainly in filtering out wrong places)
 - `length_of_parking_area` (*float*)
   - how long will be covered before cancelling searching

