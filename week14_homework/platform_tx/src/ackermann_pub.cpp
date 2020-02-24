#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Twist.h>


#define FRE 30
int main(int argc, char *argv[]){
    ros::init(argc, argv, "test_cmd_pub");
    ros::NodeHandle nh;
    ros::Publisher ackermann_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel",100);
    ros::Rate loop_rate(FRE);

    geometry_msgs::Twist msg;
    
    bool flag = true;
    while(ros::ok()){ 
        int test;
        nh.getParam("test", test);
        //msg.linear.x = test;
        msg.angular.z = 19.0/180.0*3.141592;

        ackermann_publisher.publish(msg);
        loop_rate.sleep();
    }
}