/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Int32.h"

#include <custom_msgs/Motion.h>
#include "custom_msgs/Buttons.h"
#include "custom_msgs/Indicators.h"
#include "custom_msgs/DriveCommand.h"
#include <common/state_publisher.h>

#include <stm32_bridge/usb.hpp>
#include <stm32_bridge/bridge.h>
#include <sstream>

void driveCallback(const custom_msgs::DriveCommand::ConstPtr &msg);
void indicatorsCallback(const custom_msgs::Indicators::ConstPtr &msg);

Pub_messages pub_messages;
Sub_messages sub_messages;

USB_STM Usb;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stm32_bridge");
    ros::NodeHandle n;
    ros::NodeHandle pnh("~");

    ros::Publisher motion_publisher = n.advertise<custom_msgs::Motion>("selfie_out/motion", 100);
    ros::Publisher buttons_publisher = n.advertise<custom_msgs::Buttons>("selfie_out/buttons", 100, true);
    ros::Publisher sensor_publisher = n.advertise<std_msgs::Int32>("selfie_out/sensor", 100);

    StatePublisher state_rc_publisher_("/state/rc");

    ros::Subscriber ackerman_subscriber = n.subscribe("selfie_in/drive", 1, driveCallback);
    ros::Subscriber indicators_subscriber = n.subscribe("selfie_in/indicators", 1, indicatorsCallback);

    bool first_button_state_last_time = false;
    bool second_button_state_last_time = false;
    int rc_state_last_time = 0;

    // Publish for the first time
    pub_messages.buttons_msg.is_pressed_first = false;
    pub_messages.buttons_msg.is_pressed_second = false;
    buttons_publisher.publish(pub_messages.buttons_msg);

    Usb.init();
    Time time;
    ros::Rate sleep_rate(200);

    while (ros::ok())
    {
        // if new data read - publish
        if (Usb.read_from_STM())
        {
            Usb.fill_publishers(pub_messages);

            // publishing msg
            motion_publisher.publish(pub_messages.motion_msg);

            if (pub_messages.buttons_msg.is_pressed_first !=  first_button_state_last_time ||
                pub_messages.buttons_msg.is_pressed_second !=  second_button_state_last_time)
            {
                buttons_publisher.publish(pub_messages.buttons_msg);
                first_button_state_last_time = pub_messages.buttons_msg.is_pressed_first;
                second_button_state_last_time = pub_messages.buttons_msg.is_pressed_second;
            }

            if (pub_messages.rc_state != rc_state_last_time)
            {
                rc_state_last_time = pub_messages.rc_state;
                state_rc_publisher_.updateState(pub_messages.rc_state);
            }

            sensor_publisher.publish(pub_messages.sensor);
        }

        // send subscribed data
        Usb.send_frame_to_STM(time.get_ms_time(), sub_messages);

        ros::spinOnce();
        sleep_rate.sleep();
    }
}

void driveCallback(const custom_msgs::DriveCommand::ConstPtr &msg)
{
    sub_messages.ackerman.steering_angle_front = msg->steering_angle_front;
    sub_messages.ackerman.steering_angle_rear = msg->steering_angle_rear;
    sub_messages.ackerman.speed = msg->speed;
    sub_messages.ackerman.acceleration = msg->acceleration;
}

void indicatorsCallback(const custom_msgs::Indicators::ConstPtr &msg)
{
    if (msg->is_active_left)
    {
        Usb.send_cmd_to_STM(left_indicator_on);
    }
    else
    {
        Usb.send_cmd_to_STM(left_indicator_off);
    }

    if (msg->is_active_right)
    {
        Usb.send_cmd_to_STM(right_indicator_on);
    }
    else
    {
        Usb.send_cmd_to_STM(right_indicator_off);
    }
}
