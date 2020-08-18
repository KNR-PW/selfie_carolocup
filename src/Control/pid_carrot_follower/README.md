# PID carrot follower
Consist of nodes:
- cmd_creator
- pid_controller ([link](http://wiki.ros.org/pid))
- control_calculator
- const_setpoint
## Usage
```
. devel/setup.bash
roslaunch pid_carrot_follower pid_carrot_follower_example.launch
```
## Topics
### Subscribed topics
- `/path` ([nav_msgs/Path](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float64.html))
- `/max_speed` ([std_msgs/Float64](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float64.html))
### Published topics
- `/drive/lane_control` ([ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/melodic/api/ackermann_msgs/html/msg/AckermannDriveStamped.html))

## Parameters
###
- `L` (*float*, default: 0.3)
  - position_offset + L*heading_offset
- `lookahead` (*float*, default: 0.0)
  - Distance from the car where offsets are calculated [m]
- `min_spped` (*float*, default: 0.5)
  - Min speed of the car [m/s]
-  `max_acceleration` (*float*)
   - max acceleration of the car
-  `max_curvature` (*float*)
   - max possible curvature (linear speed control)
