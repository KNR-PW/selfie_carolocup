#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "custom_msgs/Motion.h"

#include <cmath>

class Bag_convert
{
private:
  float distance;
  float yaw;
  float speed_linear;
  float speed_yaw;
  float sinr_cosp;
  float cosy_cosp;
  custom_msgs::Motion motion;
  ros::NodeHandle n;
  ros::Rate rate = ros::Rate(100);
  ros::Subscriber dist;
  ros::Subscriber imu;
  ros::Subscriber speed;
  ros::Publisher pub;

public:
  Bag_convert()
  {
    dist = n.subscribe("/distance", 10, &Bag_convert::chatterCallbackdist, this);
    imu = n.subscribe("/imu", 10, &Bag_convert::chatterCallbackimu, this);
    speed = n.subscribe("/stm32/speed", 10, &Bag_convert::chatterCallbackspeed, this);
    pub = n.advertise<custom_msgs::Motion>("/selfie_out/motion", 10);
  }
  void chatterCallbackdist(const std_msgs::Float32::ConstPtr& msg)
  {
    distance = msg->data;
  }
  void chatterCallbackspeed(const std_msgs::Float32::ConstPtr& msg)
  {
    speed_linear = msg->data;
  }
  void chatterCallbackimu(const sensor_msgs::Imu::ConstPtr& msg)
  {
    speed_yaw = msg->angular_velocity.z;
    sinr_cosp = 2 * (msg->orientation.w * msg->orientation.x + msg->orientation.x * msg->orientation.y);
    cosy_cosp = 1 - 2 * (msg->orientation.y * msg->orientation.y + msg->orientation.z * msg->orientation.z);
    yaw = std::atan2(sinr_cosp, cosy_cosp);
  }

  void run()
  {
    while (ros::ok())
    {
      motion.distance = distance;
      motion.speed_linear = speed_linear;
      motion.yaw = yaw;
      motion.speed_yaw = speed_yaw;
      pub.publish(motion);
      ros::spinOnce();
      rate.sleep();
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bag_converter");
  Bag_convert my_node;
  my_node.run();

  return 0;
}