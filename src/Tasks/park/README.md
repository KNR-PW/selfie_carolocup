# Park Action
Goes straight until the starting position is reached and parks in the parking spot. Waits two seconds and leaves the parking spot.

## Usage
```
rosrun park park
```

## Topics
### Subscribed
- `/selfie_out/motion` ([std_msgs/Float32](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32.html))
used for localization
- `/RoadLines` ([custom_msgs/RoadLines](./../../Shared/custom_msgs/msg/RoadLines.msg))
parking position is determined relative to the right lane marking
### Published
- `/drive_park` ([custom_msgs/DriveCommand](./../../Shared/custom_msgs/msg/DriveCommand.msg))
steering commands
- `/selfie_in/indicators` ([custom_msgs/Indicators](./../../Shared/custom_msgs/msg/Indicators.msg))
-  `/state/task` ([std_msgs/Int8](https://docs.ros.org/api/std_msgs/html/msg/Int8.html))
   - state of the task - see ([enums](./../../Shared/custom_msgs/include/custom_msgs/task_enum.h))

## Called Services
- `/steering_parallel` ([std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html))
changes the steering mode to parallel when starting the parking manouvre

## Parameters
- `state_msgs` (bool)
printing messages indicating the current state of the parking manouvre
- `parking_speed` (float)
speed during the parking manouvre
- `back_to_mid` (float)
distance between the base_link and the middle of the vehicle
- `idle_time` (float)
time spent idle in the parking spot
- `iter_distance` (float)
one move distance
- `angle_coeff` (float)
angle coefficient for localization (higher = parked closer to the lane)
- `max_turn` (float)
maximal wheel turn angle
- `turn_delay` (float)
time to wait for the turning direction change
- `line_dist_end` (float)
distance to the left parking spot bounding line, at which the parking manouvre is to be finished
- `start_parking_speed` (float)
the speed at which the car goes right before starting the parking manouvre
