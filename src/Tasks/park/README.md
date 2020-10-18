# Park Action
Goes straight until the starting position is reached and parks in the parking spot. Waits two seconds and leaves the parking spot.

## Usage
```
rosrun park park
```

## Topics
### Subscribed
- `/distance` ([std_msgs/Float32](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32.html))
used for localization
- `/RoadLines` ([custom_msgs/RoadLines](./../../Shared/custom_msgs/msg/RoadLines.msg))
parking position is determined relative to the right lane marking
### Published
- `/drive/park` ([custom_msgs/DriveCommand](./../../Shared/custom_msgs/msg/DriveCommand.msg))
steering commands
- `/selfie_in/indicators` ([custom_msgs/Indicators](./../../Shared/custom_msgs/msg/Indicators.msg))

## Called Services
- `/steering_parallel` ([std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html))
changes the steering mode to parallel when starting the parking manouvre

## Parameters
- `state_msgs` (bool, default=false)
printing messages indicating the current state of the parking manouvre
- `parking_speed` (float, default=0.8)
speed during the parking manouvre
- `back_to_mid` (float, default=0.18)
distance between the base_link and the middle of the vehicle
- `idle_time` (float, default=2)
time spent idle in the parking spot
- `iter_distance` (float, default=0.2)
one move distance
- `angle_coeff` (float, default=0.5)
angle coefficient for localization (higher = parked closer to the lane)
- `max_turn` (float, default=0.5)
maximal wheel turn angle
- `turn_delay` (float, default=0.1)
time to wait for the turning direction change
- `line_dist_end` (float, default=0.15)
distance to the left parking spot bounding line, at which the parking manouvre is to be finished
- `start_parking_speed` (float, default=0.5)
the speed at which the car goes right before starting the parking manouvre
