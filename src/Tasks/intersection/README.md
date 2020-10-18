# Intersection Action

Intersection action server- Controls speed of car when approached intersection

Server uses [intersection.action](./../../Shared/custom_msgs/action/intersection.action) to communicate with client

## Usage

```
source ./devel/setup.bash
roslaunch intersection intersection_example.launch
```

## Topics
### Action name
- `task/intersection`

### Subscribed topics
-  `/obstacles` ([custom_msgs/Box2DArray](./../../Shared/custom_msgs/msg/Box2DArray.msg))
   - detected obstacles
-  `/intersection/stop` ([custom_msgs/IntersectionStop](./../../Shared/custom_msgs/msg/IntersectionStop.msg))
   - nearest intersection information
-  `/selfie_out/motion` ([custom_msgs/Motion](./../../Shared/custom_msgs/msg/Motion.msg))
   - selfie motion

### Published topics
-  `/max_speed` ([std_msgs/Float64](https://docs.ros.org/api/std_msgs/html/msg/Float64.html))
   - current speed of car
-  `/visualization/intersection` ([visualization_msgs/Marker](https://docs.ros.org/api/visualization_msgs/html/msg/Marker.html))
   - (only when parameter `visualization=true` visualizes found obstacles)

## Parameters
-  `distance_to_intersection` (*float*)
   - how far from intersection car should stop
-  `road_width` (*float*)
-  `point_min_y`,`point_max_y`(*float*)
   - describing width of area of interest (height depends on `distance_to_intersection` and `road_width`)
-  `stop_time` (*float*)
   - how long car should stop on intersection (even if there are no other cars)
-  `speed_default` (*float*)
   - default speed of approaching to intersection
-  `distance_of_blind_approaching` (*float*)
   - distance which skould covered blindly (to reduce distance to intersection)
-  `num_corners_to_detect` (*int*)
   - number of corners which should be found in area on interest
-  `visualization` (*bool*, default: false)
   - Whether or not visualization topics are active
