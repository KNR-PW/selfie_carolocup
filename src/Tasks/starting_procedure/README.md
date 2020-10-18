# starting_procedure
Perform starting procedure. Wait for button press and start searching for the gate and qr code. Starts When either of them is found or a button is pressed. The last buttonpress decides the competition.
Package implements action `starting_procedure` triggered by scheduler.

## starting_procedure
Controls the starting procedure

### Usage
```
roslaunch starting_procedure starting_procedure_example.launch
```
### Subscribed topics
- `/selfie_out/buttons` ([custom_msgs/Buttons](./../../Shared/custom_msgs/msg/Buttons.msg))
   buttons state of the car, used to choose the competition
- `/selfie_out/motion` ([custom_msgs/Motion](./../../Shared/custom_msgs/msg/Motion.msg))
   Distance covered by the car (based on encoder's reading) used to determine when to stop driving straightforward
- `/odom` ([nav_msgs/Odometry](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html))
   feedback to the proportional regulator
### Published topics
- `/drive/starting_procedure` ([custom_msgs/DriveCommand](./../../Shared/custom_msgs/msg/DriveCommand.msg))
   Drive commands for uC to run the car

### Called services
- `/startQrSearch` ([std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html))
   Called when a button is pressed, starts searching for the qr code
- `/stopQrSearch` ([std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html))
   Called when a button is pressed again, indicates the search os no longer necessary
- `/startGateScan` ([std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html))
   Called when a button is pressed, starts searching for the gate with the lidar sensor

### Parameters
- `starting_speed` (float, default=2)
- `use_scan` (bool, default=false)
   whether to start when the lidar stops detecting the gate
- `use_qr` (bool, default=true)
   whether to start when the camera stops detecting the qr code
- `Kp` (float, default=1)
   the proportional gain of starting regulator, keeps the direction throughout the starting procedure

## Qr Decoder
Searches for the qr code, informs whenever the qr code stops being detected.
### Usage
```
rosrun starting_procedure qr_decoder
```

### Published topics
- `/qr_gate_open` ([std_msg/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html))
   indicates the starting gate was opened
### Subscibed topics
- `/camera/image_rect` ([sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html))
   used to detect the qr code

### Advertised services
- `startQrSearch` ([std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html))
   Starts the search for the qr code
- `stopQrSearch` ([std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html))
   Stops the search for the qr code
### Parameters
- `min_detect_rate` (float, default=0.4)
   the rate of detection that needs to be exceeded 
- `iterations_to_valid` (int, default=2)
   minimal number of timer iterations with detect rate over the threshold in order to validate the detection of the qr code
- `visualization` (bool, default=false)
   open the visualization of the qr code detection

## Gate scanner
Searches for the starting gate with the laser scans, and informs whenever the gate stops being detected.
### Usage
```
rosrun starting_procedure gate_scanner
```

### Published topics
- `/scan_gate_open` ([std_msg/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html))
   indicates the starting gate was opened
### Subscibed topics
- `/scan` ([sensor_msgs/LaserScan](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html))
   used to detect the gate

### Advertised services
- `startGateScan` ([std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html))
   starts searching for the gate with the lidar sensor when called

### Parameters
- `no_obstacle_time_thresh` (float, default=1.5)
   when there are no detections throughout this amount of time, the feedback is sent
- `min_distance` (float, default=0.1)
   start distance of ROI
- `max_distance` (float, default=0.5)
   end distance of ROI
- `min_width` (float, default=0.2)
   width of ROI
- `min_gate_seen_count` (float, default=5)
   minimal amount of detections to send the feedback
