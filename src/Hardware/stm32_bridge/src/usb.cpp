/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#include <stm32_bridge/usb.hpp>

int USB_STM::init(int speed)
{
    char port[] = "/dev/serial/by-id/usb-KNR_Selfie_F7_00000000001A-if00";
    // char port[] = "/dev/serial/by-id/STM32F407";
    fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        ROS_ERROR("Could not open serial communication on port!.\n"
                  "Make sure you did: chmod u +rw /dev/serial/by-id/usb-KNR_Selfie_F7_00000000001A-if00");
        return -1;
    }
    else
    {
        ROS_INFO("Opened serial communication on port\n");
    }

    // Get attributes of transmission
    struct termios tty;
    if (tcgetattr(fd, &tty) < 0)
    {
        ROS_ERROR("Error while getting attributes!");
        return -2;
    }

    // Set input and output speed
    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);

    tty.c_cflag |= (CLOCAL | CREAD);  // program will not become owner of port
    tty.c_cflag &= ~CSIZE;            // bit mask for data bits
    tty.c_cflag |= CS8;               // 8 bit data lenght
    tty.c_cflag |= PARENB;            // enable parity
    tty.c_cflag &= ~PARODD;           // even parity
    tty.c_cflag &= ~CSTOPB;           // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;          // no hardware flowcontrol

    // non-canonical mode
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    // fetch bytes asap
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    // Set new parameters of transmission
    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        ROS_ERROR("Error while setting attributes!");
        return -3;
    }

    // send ACK command
    unsigned char data_enable[3] = {frame_startbyte, cmd_data_enable, frame_endbyte};
    write(fd, data_enable, 3);

    return 1;
}
void USB_STM::send_cmd_to_STM(uint8_t cmd)
{
    unsigned char cmd_data[3] = {frame_startbyte, cmd, frame_endbyte};
    write(fd, cmd_data, 3);
}

bool USB_STM::read_from_STM()
{
    int read_state = read(fd, &read_buffer[0], 512);

    // cast read_buffer to frame
    read_frame = (UsbReadFrame_s *)read_buffer; // NOLINT

    if (read_state == USB_RECEIVE_SIZE && read_frame->start_code == frame_startbyte &&
        read_frame->length == USB_RECEIVE_SIZE && read_frame->end_code == frame_endbyte)
    {
        return true;  // correct data
    }
    else
    {
        return false;  // no new data
    }
}

void USB_STM::send_frame_to_STM(uint32_t timestamp_ms, Sub_messages to_send)
{
    send_frame = (UsbSendFrame_s *)send_buffer; // NOLINT
    send_frame->start_code = frame_startbyte;
    send_frame->length = USB_SEND_SIZE;

    send_frame->timecode = timestamp_ms;
    send_frame->steering_fi_front = (int16_t)(to_send.ackerman.steering_angle_front * 10000);
    send_frame->steering_fi_back = (int16_t)(to_send.ackerman.steering_angle_rear * 10000);
    send_frame->speed = (int16_t)(to_send.ackerman.speed * 1000);
    send_frame->acceleration = (int16_t)(to_send.ackerman.acceleration * 1000);
    send_frame->jerk = (int16_t)(to_send.ackerman.jerk * 1000);
    send_frame->end_code = frame_endbyte;

    write(fd, &send_buffer, USB_SEND_SIZE);
}

void USB_STM::fill_publishers(Pub_messages &pub_data)
{
    // Calculate yaw from quaternion
    float quat_x = static_cast<float>(read_frame->x / 32767.f);
    float quat_y = static_cast<float>(read_frame->y / 32767.f);
    float quat_z = static_cast<float>(read_frame->z / 32767.f);
    float quat_w = static_cast<float>(read_frame->w / 32767.f);
    tf::Quaternion q(quat_x, quat_y, quat_z, quat_w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    pub_data.motion_msg.header.frame_id = "base_link";
    pub_data.motion_msg.header.stamp = ros::Time::now();
    pub_data.motion_msg.yaw = yaw;
    pub_data.motion_msg.distance = static_cast<float>(read_frame->distance) / 1000.0;
    pub_data.motion_msg.speed_yaw = static_cast<float>(read_frame->rates[2] / 65.535f);
    pub_data.motion_msg.speed_linear = static_cast<float>(read_frame->velocity) / 1000.0;

    if ((read_frame->buttons >> 0) & 1U)
    {
        pub_data.buttons_msg.is_pressed_first = true;
    }
    else
    {
        pub_data.buttons_msg.is_pressed_first = false;
    }
    if ((read_frame->buttons >> 2) & 1U)
    {
        pub_data.buttons_msg.is_pressed_second = true;
    }
    else
    {
        pub_data.buttons_msg.is_pressed_second = false;
    }

    // reset states
    pub_data.rc_state = read_frame->futaba_state + 1;

    // pub_data.sensor.data = read_frame->sensor;
}

USB_STM::~USB_STM()
{
    send_frame = (UsbSendFrame_s *)send_buffer; // NOLINT
    send_frame->start_code = frame_startbyte;
    send_frame->length = USB_SEND_SIZE;

    send_frame->timecode = 0;
    send_frame->steering_fi_back = 0;
    send_frame->steering_fi_front = 0;
    send_frame->speed = 0;
    send_frame->acceleration = 0;
    send_frame->jerk = 0;
    send_frame->end_code = frame_endbyte;

    write(fd, &send_buffer, USB_SEND_SIZE);
}
