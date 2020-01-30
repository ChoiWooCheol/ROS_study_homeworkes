#include<ros/ros.h>
#include"turtlesim/Pose.h"


void turtleCallback(const turtlesim::Pose::ConstPtr& msg){
    ROS_INFO("turtle x                = %f", msg->x);
    ROS_INFO("turtle y                = %f", msg->y);
    ROS_INFO("turtle theta            = %f", msg->theta);
    ROS_INFO("turtle linear_velocity  = %f", msg->linear_velocity);
    ROS_INFO("turtle angular_velocity = %f", msg->angular_velocity);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "turtlesim_view");
    ros::NodeHandle nh;
    ros::Subscriber turtlesim_sub = nh.subscribe("turtle1/pose", 100, &turtleCallback);

    ros::spin();
    return 0;
}