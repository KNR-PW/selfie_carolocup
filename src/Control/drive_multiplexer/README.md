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
- `/drive/lane_control` ([custom_msgs/DriveCommand](./../../Shared/custom_msgs/msg/DriveCommand.msg))
- `/drive/park` ([custom_msgs/DriveCommand](./../../Shared/custom_msgs/msg/DriveCommand.msg))
- `/drive/starting_procedure` ([custom_msgs/DriveCommand](./../../Shared/custom_msgs/msg/DriveCommand.msg))
### Published topics
- `/selfie_in/drive` ([custom_msgs/DriveCommand](./../../Shared/custom_msgs/msg/DriveCommand.msg))

## Services
###
- `/drive_multiplexer/select`
  - Select an input topic to output
- `/drive_multiplexer/add`
  - Add a new input topic
- `/drive_multiplexer/delete`
  - Delete an input topic