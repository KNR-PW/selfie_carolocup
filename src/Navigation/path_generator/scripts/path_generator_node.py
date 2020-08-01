#!/usr/bin/env python
'''
*Copyright ( c ) 2020, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
'''

import rospy

from math import atan, cos, sin
from numpy.polynomial.polynomial import Polynomial

from custom_msgs.msg import RoadMarkings
from std_msgs.msg import Float64
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from dynamic_reconfigure.server import Server
from path_generator.cfg import PathGeneratorConfig


class PathGenerator:

    def __init__(self):
        self.interval_x = rospy.get_param('~interval_x', 0.1)
        self.max_distance = rospy.get_param('~max_distance', 1.0)
        self.path_offset = 0
        self.dr_srv = Server(PathGeneratorConfig, self.reconfigure_callback)
        self.path_pub = rospy.Publisher('path', Path, queue_size=1)
        self.road_markings_sub = rospy.Subscriber('road_markings',
                                                  RoadMarkings,
                                                  self.road_markings_callback,
                                                  queue_size=1)
        self.max_speed_sub = rospy.Subscriber('path_offset',
                                              Float64,
                                              self.path_offset_callback,
                                              queue_size=1)

        print("max_distance set to: " + str(self.max_distance))
        print("interval_x set to: " + str(self.interval_x))

    def road_markings_callback(self, msg):
        c_poly = Polynomial(msg.center_line)
        path = Path()
        path.header = msg.header

        x = 0.0
        while x < self.max_distance:
            der = c_poly.deriv()(x)
            angle = atan(der)
            x_offset = x + self.path_offset*sin(angle)
            y_offset = c_poly(x) - self.path_offset*cos(angle)

            path_pose = PoseStamped()
            path_pose.header = msg.header
            path_pose.pose.position.x = x_offset
            path_pose.pose.position.y = y_offset
            path_pose.pose.orientation.z = angle

            path.poses.append(path_pose)

            x += self.interval_x

        self.path_pub.publish(path)

    def path_offset_callback(self, msg):
        self.path_offset = msg.data

    def set_if_reconfigured(self, config):
        for it in config.keys():
            if it is 'groups':
                continue
            if self.__dict__[it] != config[it]:
                self.__dict__[it] = config[it]
                rospy.loginfo("%s new value is %f", it, config[it])

    def reconfigure_callback(self, config, level):
        self.set_if_reconfigured(config)
        return config


if __name__ == '__main__':
    rospy.init_node('path_generator')
    PathGenerator()
    rospy.spin()
