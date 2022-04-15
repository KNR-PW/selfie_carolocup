# sign_detector

sign_detector- detects signs and provides information if the vehicle is in limitation zone (speed or overtaking).

## Usage

```bash
. devel/setup.bash
roslaunch roslaunch sign_detector sign_detector_example.launch
```

## Topics

### Subscribed topics

- `/camera_basler/image_rect` ([sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html))
  - grayscale camera image
- `/selfie_out/motion` ([custom_msgs/Motion](./../../Shared/custom_msgs/msg/Motion.msg))
  - distance covered by car

### Published topics

- `/can_overtake` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))
  - describes state whether car is allowed to overtake
- `/speed_limit` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))
  - describes state whether car is obliged to slow down

## Parameters

- `visualization` (_bool_, default: true)
  - Whether or not to visualize sign detection
- `speed_limit_distance` (_float_)
  - The distance of speed limit after passing the sign (after the last frame with sign detected + `activation_time`)
- `overtaking_ban_distance` (_float_)
  - The distance of overtaking ban after passing the sign (after the last frame with sign detected + `activation_time`)
- `activation_time` (_float_)
  - The time after which the limits are introduced
