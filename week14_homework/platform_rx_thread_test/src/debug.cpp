#include <ros/ros.h>
#include <serial/serial.h>
#include <mutex>
#include <thread>
#include <deque>
#include <exception>
#include <ctime>
#include<cmath>
#include<cstdlib>
#include<vector>

#include "platform_rx_msg/platform_rx_msg.h"

void callback(const platform_rx_msg::platform_rx_msg::ConstPtr& msg){
    ROS_INFO("speed        : %f", msg->speed);
    ROS_INFO("steer        : %f", msg->steer);
    ROS_INFO("brake        : %f", msg->brake);
    ROS_INFO("seq          : %f", msg->seq);
    ROS_INFO("speed filter : %f", msg->speedfilter);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "debug_sb");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("raw/platform_rx",100,callback);

    ros::spin();
}