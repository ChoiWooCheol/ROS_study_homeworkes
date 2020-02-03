#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

double width_robot = 0.15;
double wheel_radius = 0.05;

double linear_velocity = 0;
double angular_velocity = 0;

void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
	geometry_msgs::Twist twist = twist_aux;	
	linear_velocity = twist_aux.linear.x;
	angular_velocity = twist_aux.angular.z;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "state_publisher");
	ros::NodeHandle n;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
	ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 10, cmd_velCallback);

	// initial position
	double x = 0.0; 
	double y = 0.0;
	double th = 0;

	ros::Time current_time;
	ros::Time last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(20);

	const double degree = M_PI/180;

	// message declarations
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";

	while (ros::ok()) {
		current_time = ros::Time::now(); 

		double dt = (current_time - last_time).toSec();
        double delta_s = linear_velocity*dt;
        double delta_th = angular_velocity*dt;
                /* odometry   계산 코드 */	
		
		th += delta_th;
		x += delta_s * cos(th);
		y += delta_s * sin(th);
				

		geometry_msgs::Quaternion odom_quat;	
		odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);

		// update transform
		odom_trans.header.stamp = current_time; 
		odom_trans.transform.translation.x = x; 
		odom_trans.transform.translation.y = y; 
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

		//filling the odometry
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_footprint";

		// position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0;
		odom.pose.pose.orientation  = tf::createQuaternionMsgFromYaw(th);

		//velocity
		odom.twist.twist.linear.x = linear_velocity * cos(th);
		odom.twist.twist.linear.y = linear_velocity * sin(th);
		odom.twist.twist.angular.z = angular_velocity;

		// publishing the odometry and the new tf
		broadcaster.sendTransform(odom_trans);
		odom_pub.publish(odom);
		ros::spinOnce();
		loop_rate.sleep();

		last_time = current_time;
	}
	return 0;
}
