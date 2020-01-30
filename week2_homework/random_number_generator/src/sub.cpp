#include<ros/ros.h>
#include"random_number_generator/rn.h"

int alive = 0;
int sum = 0;
void randCallback(const random_number_generator::rn::ConstPtr& msg){
    ++alive;
    sum += msg->randint;
    ROS_INFO("sum     = %d", sum);
    ROS_INFO("alive   = %d", alive);
    ROS_INFO("limit   = %d", msg->limit);
    ROS_INFO("randint = %d", msg->randint);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rn_sub");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("rn_info", 100, &randCallback);

    ros::spin();
        
    return 0;
}