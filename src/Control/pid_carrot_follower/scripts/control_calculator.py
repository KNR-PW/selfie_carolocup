#!/usr/bin/env python3

import rospy
import math

from std_msgs.msg import Float64
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from dynamic_reconfigure.server import Server

from pid_carrot_follower.cfg import ControlCalculatorConfig

position_offset = 0.0
heading_offset = 0.0
speed = 0.0
max_acceleration = 1.0


def config_callback(config, level):
    for it in config.keys():
        if it is 'groups':
            continue
        if globals()[it] != config[it]:
            globals()[it] = config[it]
            rospy.loginfo("%s new value is %f", it, config[it])
    return config


def path_callback(msg):
    global look_ahead
    global heading_offset
    global position_offset
    if msg.poses[len(msg.poses) - 1].pose.position.x < look_ahead:
        print(msg.poses[len(msg.poses) - 1].pose.position.x)
        print(look_ahead)
        raise ValueError("Look_ahead is greater than path range")

    pose_next = Pose()
    for p in msg.poses:
        if p.pose.position.x >= look_ahead:
            pose_next = p.pose
            break

    position_offset = -pose_next.position.y
    heading_offset = -pose_next.orientation.z


def max_speed_callback(msg):
    global speed
    global min_speed
    global max_curvature
    global heading_offset
    speed = heading_offset * (min_speed - msg.data) / max_curvature + msg.data


if __name__ == '__main__':
    rospy.init_node('control_calculator')

    L = rospy.get_param('~L', 0.3)
    look_ahead = rospy.get_param('~look_ahead', 0.3)
    publish_rate = rospy.get_param('~publish_rate', 50)
    max_acceleration = rospy.get_param('~max_acceleration', 3)
    max_curvature = rospy.get_param('~max_curvature', 1.8)
    min_speed = rospy.get_param('~min_speed', 1.0)
    print("L set to: " + str(L))
    print("look_ahead set to: " + str(look_ahead))
    print("publish_rate set to: " + str(publish_rate))
    print("max_acceleration set to: " + str(max_acceleration))
    print("max_curvature set to: " + str(max_curvature))
    print("min_speed set to: " + str(min_speed))

    srv = Server(ControlCalculatorConfig, config_callback)

    path_sub = rospy.Subscriber('path', Path, path_callback, queue_size=1)

    max_speed_sub = rospy.Subscriber('max_speed',
                                     Float64,
                                     max_speed_callback,
                                     queue_size=1)

    combined_offset_pub = rospy.Publisher('combined_offset',
                                          Float64,
                                          queue_size=1)
    speed_pub = rospy.Publisher('speed', Float64, queue_size=1)
    acceleration_pub = rospy.Publisher('acceleration', Float64, queue_size=1)

    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
        combined_offset = position_offset + L * math.sin(heading_offset)
        combined_offset_pub.publish(combined_offset)

        speed_pub.publish(speed)

        acceleration_pub.publish(max_acceleration)
        rate.sleep()
