# Schedulerpy

Python reimplementation of legacy scheduler.

## `scheduler`

Package used to administer switching actions (scenarios).
![scheduler_flow_chart](https://user-images.githubusercontent.com/26739110/75875922-f28e7280-5e14-11ea-848b-db97cc750ab8.PNG)

## Action clients

Action clients enable starting and stopping action servers placed in other packages.
Common methods for action clients have been implemented as interface class in file `client_interface.h`.
In current implementation following clients are used:

- starting_action_client
- drive_action_client
- search_ation_client
- park_action_client
- intersection_action_client

### Subscribed topics

`/state/rc` ([std_msgs/Int8](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int8.html))

- state of RC task - see ([enums](./../../Shared/custom_msgs/include/custom_msgs/rc_enum.h))

### Published topics

- `/state/task` ([std_msgs/Int8](https://docs.ros.org/api/std_msgs/html/msg/Int8.html))
- state of the task - see ([enums](./../../Shared/custom_msgs/include/custom_msgs/task_enum.h))

## Parameters

`~starting_distance` (_float_, default: 1.0)
Distance car should cover to drive out of the starting box (m)

`~parking_spot` (_float_, default: 0.5)
Minimum parking spot width (m)

`~num_park_to_complete` (_int_, default: 2)
Number of successful parkings
