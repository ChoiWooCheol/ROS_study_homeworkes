#include<ros/ros.h>
#include<std_msgs/String.h>
//#include"hello_world_node/jaemin.h"
#include<sstream>
/*
int main(int argc, char** argv)
{
	ros::init(argc, argv, "hello_world_node");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<std_msgs::String>("TopicType", 1000);
	ros::Rate loop_late(10);
	int count = 0;
	while(ros::oK()){
		std_msgs::String msg;
		std::stringstream ss;
		ss<<"hello world!"<<count;
		msg.data = ss.c_str();
		ROS_INFO("%s",msg.data.c_str())

		pub.publisher(msg);
		ros::spin();
		loop_rate.sleep();
		++count;
	}
	return 0;
}
*/
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "hello_world_node");
	ros::NodeHandle nh;
	ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("say_hello_world", 1000);
	ros::Rate loop_rate(10);
	int count = 0;
	while(ros::ok())
	{
		std_msgs::String msg;
		std::stringstream ss;
		ss<<"hello world!"<<count;
		msg.data=ss.str();
		ROS_INFO("%s", msg.data.c_str());
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	
	return 0;
}
