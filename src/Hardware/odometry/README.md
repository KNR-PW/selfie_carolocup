# odometry

`odometry` package provides a node with the same name responsible for calculating odometry from encoder & IMU data. Results are published both in the `odom` topic and as a [tf2](http://wiki.ros.org/tf2) transformation.

## odometry

### Subscribed topics


- `selfie_out/motion` ([custom_msgs/Motion](./../../Shared/custom_msgs/msg/Motion.msg))
  - motion description data

### Published topics

- `odom` ([nav_msgs/Odometry](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html))
  - odometry data

### Called services
- `/reset/odom` ([std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html))
  - Reset odometry

## Parameters

- `~rear_axis_frame` (*string*, default: base_link)
  - The name of the rear axis frame.

## Provided tf transforms

- `odom` → `~rear_axis_frame`
