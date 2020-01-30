#include<ros/ros.h>
#include"beginner_tutorials/simple_msg.h"

void chatterCallback(const beginner_tutorials::simple_msgConstPtr& ptr){
	ROS_INFO("I heard from %s : [%s]", ptr->id.c_str(),ptr->data.c_str());
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "listener");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("chatter", 100 , &chatterCallback);
	ros::spin();

	return 0;
}
