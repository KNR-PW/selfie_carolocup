/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/ 

#include <ros/ros.h>

#include <scheduler/scheduler.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scheduler");
    Scheduler selfie_scheduler;  // action initialization
    selfie_scheduler.init();
    ros::Rate sleep_rate(100);

    // wait for key press
    selfie_scheduler.waitForStart();
    while (ros::ok())
    {
        ros::spinOnce();
        selfie_scheduler.loop();
        sleep_rate.sleep();
    }
}
