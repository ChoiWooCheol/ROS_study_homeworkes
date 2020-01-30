#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_broadcaster.h>
#include<string>
#include<cmath>
#include<gazebo_msgs/LinkStates.h>
#include<geometry_msgs/Twist.h>
#include<vector>
#include<iostream>

#define radi 3.14159265359 / 180

using namespace std;

class DiffDriveOdometry{
public:
    DiffDriveOdometry():seq(0),
                        x(0.0), 
                        y(0.0), 
                        theta(0.0), 
                        last(ros::Time::now()),
                        private_nh("~") {
        sub = nh.subscribe("/gazebo/link_states", 100, &DiffDriveOdometry::calcWheelVelocityGazeboCB, this);
        odomPub = nh.advertise<nav_msgs::Odometry>("/custom_odom",100);
        /*
        
        */
    }

    void calcWheelVelocityGazeboCB(const gazebo_msgs::LinkStates::ConstPtr& ptr){
        /*
            calculate velocity of wheel_1, wheel_3
        */
    }

    void boradcastTransform(){

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x,y,0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, theta);
        transform.setRotation(q);

        // linking custom_base from custom_odom
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), odom_link_id.c_str(), base_link_id.c_str()));

        /*
            boradcastTransform() 함수
            line-by-line 코드분석
        */
    }

    void pubTF(){
        ros::Time cur = ros::Time::now();
        double dt = (cur - last).toSec();

        /*
            odometry 계산
            * 적분식을 코드로 구현한다. dt를 구해야함.
            * dt와 강의자료의 공식을 이용하여 theta_dot, x_dot, y_dot, theta, x, y 를 구한다.
            * 위의 여섯가지 요소를 구할 때 R은 1로 계산.
            * L = 자동차 바퀴사이의 값 = separation_length
            *
        */

        nav_msgs::Odometry odom;
        odom.header.seq             = seq++;
        odom.header.stamp           = cur;
        odom.header.frame_id        = odom_link_id;
        odom.child_frame_id         = base_link_id;
        odom.pose.pose.position.x   = x;
        odom.pose.pose.position.y   = y;
        odom.pose.pose.position.z   = 0;
        odom.pose.pose.orientation  = tf::createQuaternionMsgFromYaw(theta);
        odom.twist.twist.linear.x   = x_dot;
        odom.twist.twist.linear.y   = y_dot;
        odom.twist.twist.angular.z  = theta_dot;

        odomPub.publish(odom);
        boradcastTransform();
    }

private:
    ros::NodeHandle nh, private_nh;
    ros::Subscriber sub;
    ros::Publisher odomPub;
    ros::Time last;
    string base_link_id, odom_link_id, wheel_1_id, wheel_3_id;
    double separation_length;
    double x_dot, y_dot, theta_dot;
    double x, y, theta;
    int seq;
    tf::TransformBroadcaster br;
    double left_vel, right_vel, base_vel;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mobile_robot_odometry");
    DiffDriveOdometry Odom;
    ros::Rate loop_rate(100); // dt is always 0.01s

    while(ros::ok()){
        ros::spinOnce();
        Odom.pubTF();
        loop_rate.sleep();
    }
    return 0;
}