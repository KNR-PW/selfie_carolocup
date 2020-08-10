# drive_multiplexer

Consist of node:
- mux ([link](http://wiki.ros.org/topic_tools/mux))

## Usage
```
. devel/setup.bash
roslaunch drive_multiplexer drive_multiplexer_example.launch
```
## Topics
### Subscribed topics
- `/drive/lane_control` ([ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/melodic/api/ackermann_msgs/html/msg/AckermannDriveStamped.html))
- `/drive/park` ([ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/melodic/api/ackermann_msgs/html/msg/AckermannDriveStamped.html))
- `/drive/starting_procedure` ([ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/melodic/api/ackermann_msgs/html/msg/AckermannDriveStamped.html))
### Published topics
- `/drive/out` ([ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/melodic/api/ackermann_msgs/html/msg/AckermannDriveStamped.html))

## Services
###
- `/drive_multiplexer/select`
  - Select an input topic to output
- `/drive_multiplexer/add`
  - Add a new input topic
- `/drive_multiplexer/delete`
  - Delete an input topic