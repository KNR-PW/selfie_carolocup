# `selfie_basler_fisheye`

Configuration package to launch basler camera with fisheye lens. It also contains image processing for elimnate distortion.

ROS packages included:
- pylon_camera: the driver itself
- camera_control_msgs: message and service definitions for interacting with the camera driver
- image_proc_fisheye: ROS nodelet for image rectification using OpenCV
- selfie_basler_fisheye: package for selfie configuration

### Published topics
- `camera_basler/image_rect` ([sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html))
  - Image after rectification

### Parameters
See included packages README files