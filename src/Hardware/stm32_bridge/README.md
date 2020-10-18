# STM32 Bridge

`stm32_bridge` package provides a node with the same name responsible for communication with on-board STM32 microcontroller that handles IMU, encoders and vehicle control. The node communicates with ROS using standarized message types, as described below.

## `stm32_bridge`

## Subscribed topics

`selfie_in/drive` ([custom_msgs/DriveCommand](./../../Shared/custom_msgs/msg/DriveCommand.msg))
Steering commands to be applied.

`selfie_in/indicators` ([custom_msgs/Indicators](./../../Shared/custom_msgs/msg/Indicators.msg))
Status of turn indicators.

## Published topics

`selfie_out/motion` ([custom_msgs/Motion](./../../Shared/custom_msgs/msg/Motion.msg))
Motion of the car.

`selfie_out/buttons` ([custom_msgs/Buttons](./../../Shared/custom_msgs/msg/Buttons.msg))
Buttons press info - publish when any is pressed

`switch_state` ([std_msgs/UInt8](http://docs.ros.org/api/std_msgs/html/msg/UInt8.html))
Car drive mode set by user on RC.
2 - manual, 1- semi-autonomous 0 - autonomous mode
