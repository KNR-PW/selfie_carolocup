# PID carrot follower
Consist of nodes:
- cmd_creator
- pid_controller ([link](http://wiki.ros.org/pid))
- control_calculator
- const_setpoint
- pid_tuner
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
- `/drive/lane_control` ([custom_msgs/DriveCommand](./../../Shared/custom_msgs/msg/DriveCommand.msg))

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
-  `pid_tuner_disabled` (*bool*)
   - whether or not PID tuner is activated
-  `L_Kp` (*float*)
   - Kp for low range of speed
-  `L_Ki` (*float*)
   - Ki for low range of speed
-  `L_Kd` (*float*)
   - Kd for low range of speed
-  `M_Kp` (*float*)
   - Kp for medium range of speed
-  `M_Ki` (*float*)
   - Ki for medium range of speed
-  `M_Kd` (*float*)
   - Kd for medium range of speed
-  `H_Kp` (*float*)
   - Kp for high range of speed
-  `H_Ki` (*float*)
   - Ki for high range of speed
-  `H_Kd` (*float*)
   - Kd for high range of speed
-  `LaneChange_Kp` (*float*)
   - Kp for lane change manuver
-  `LaneChange_Ki` (*float*)
   - Ki for lane change manuver
-  `LaneChange_Kd` (*float*)
   - Kd for lane change manuver
-  `M_speed` (*float*)
   - lower bound for medium range of speed [m/s]
-  `H_speed` (*float*)
   - lower bound for low range of speed [m/s]
-  `speed_change_treshold` (*float*)
   - the smallest change of speed for PID modification to be considered [m/s]