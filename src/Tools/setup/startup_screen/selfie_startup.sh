#!/usr/bin/env bash
selfie_ip="10.10.1.1"
selfie_name="selfie"

screen -S selfie  -dm

screen -S selfie -X stuff "export ROS_MASTER_URI=http://${selfie_ip}:11311\n"
screen -S selfie -X  stuff "export ROS_IP=${selfie_ip}\n"

screen -S selfie -X  stuff "cd /home/selfie/carolocup_2021\n"
screen -S selfie -X stuff "source devel/setup.bash\n"
screen -S selfie -X stuff "roslaunch startup all.launch\n"
