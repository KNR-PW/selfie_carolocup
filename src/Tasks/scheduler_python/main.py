#!/usr/bin/env python
import rospy


from scheduler_python import Scheduler



if __name__ == "__main__":
    scheduler = Scheduler()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        scheduler.loop()
        rate.sleep()

