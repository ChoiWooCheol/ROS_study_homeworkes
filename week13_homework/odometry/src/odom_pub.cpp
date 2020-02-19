#include "odometry/ackermann.h"
using namespace odometry_ackermann;

int main(int argc, char** argv){
  ros::init(argc, argv, "Odometry_Publisher");
  ros::NodeHandle nh;

  ros::Time now = ros::Time::now();

  Odometry odom;

  odom.init(now);
  odom.Odom_Transform(now);                              // odometry transform 계산
  odom.Odom_Set(now); 

  ros::Subscriber sub = nh.subscribe("raw/platform_rx", 100, &Odometry::callback, &odom);

  ros::Rate loop_rate(10);

  while(ros::ok()){
    ros::spinOnce();
    odom.publish();
    loop_rate.sleep();
  }
  return 0;
}