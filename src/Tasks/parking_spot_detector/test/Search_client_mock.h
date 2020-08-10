#pragma once

#include <common/obstacle_box.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Polygon.h>

#include <ros/ros.h>
#include <custom_msgs/searchAction.h>
#include <custom_msgs/enums.h>
#include <std_msgs/Float64.h>
#include <stdlib.h>

void topicCallback(const std_msgs::Float64ConstPtr &msg);
void feedbackCb(const custom_msgs::searchFeedbackConstPtr &feedback);
void sendMockObstacles(const ros::TimerEvent &);

class Search_client_mock
{
public:
  static float last_speed_;
  void send_goal(const float);
  Search_client_mock(const ros::NodeHandle &nh);
  static ros::Publisher obstacles_pub_;

private:
  void activeCb() { ac_.waitForServer(); }
  void doneCb(const actionlib::SimpleClientGoalState &state, const custom_msgs::searchResultConstPtr &result);
  actionlib::SimpleActionClient<custom_msgs::searchAction> ac_;
  bool send_mock_obstacles_;

  ros::NodeHandle nh_;
  ros::Subscriber speed_subscriber_;
};
