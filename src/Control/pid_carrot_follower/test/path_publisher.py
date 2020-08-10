#!/usr/bin/env python

import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('path_publisher')

    pub = rospy.Publisher('/path', Path, queue_size=1)

    rate = rospy.Rate(10)

    path = Path()
    for i in range(10):
        pose = PoseStamped()
        pose.pose.position.x = i / 10.0
        pose.pose.position.y = i
        pose.pose.orientation.z = i * 10
        path.poses.append(pose)

    while not rospy.is_shutdown():
        pub.publish(path)
        rate.sleep()
