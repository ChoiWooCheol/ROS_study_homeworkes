#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include "platform_rx_msg/platform_rx_msg.h"

namespace odometry_ackermann
{
class Odometry{
public:
    /*
    <Odometry::Odometry() 생성자>
        아무것도 안함
    */
    Odometry();



    /*
     <callback 함수>
        1. Calc_Odom 함수를 통해 Odometry 계산
        2. Odom_Transform을 통해 'odom' frame의 transform정보를 odom_trans객체에 저장
        3. odom_trans 객체를 tf에 broadcast
        4. Odom_Set 함수를 통해 Odometry를 odom객체에 저장
        5. '/odom' topic으로 odom객체를 publish
    */
    void callback(const platform_rx_msg::platform_rx_msg::ConstPtr& PlatformRX_data);
    
    
      
    /*
     <init 함수>
        처음에 시간 초기화
    */
    void init(const ros::Time& time);



    /*
     <UpdateParams 함수>
        Update parameters
    */
    void UpdateParams(void);

    

    /*
     <Calc_Odom 함수>
        Car-like Robot의 Odometry를 계산수
    */
    bool Calc_Odom(const platform_rx_msg::platform_rx_msg::ConstPtr& PlatformRX_data,
                   const ros::Time& time);



    /*
     <Odom_Set 함수>
        odom을 참조하여 Odom_Set 함수를 통해 odometry 정보를 저장
    */
    void Odom_Set(const ros::Time& time);



    /*
     <Odom_Transform 함수>
        odom_trans을 참조하여 Odom_Transform 함수를 통해 transform 정보를 저장
    */
    void Odom_Transform(const ros::Time& time);



    /*
     <sendTransform 함수>
        odom_trans_를 broadcast하는 함수 (sendTransform)
    */
    void sendTransform();



    /*
     <publish 함수>
        odom_를 publish하는 함수
    */
    void publish();


/*---  topic(sub/pub)관련 객체 선언 및 Odometry Calculation에 이용할 변수 선언  ---*/
private:
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Time timestamp_;
    
    tf::TransformBroadcaster odom_broadcaster_;       // tf broadcast
    nav_msgs::Odometry odom_;                         // publish할 odometry
    geometry_msgs::TransformStamped odom_trans_;      // tf으로 날릴 odometry transform


    std::string frame_id_, child_frame_id_;

    double wheelbase_, curvature_, ignore_up_, ignore_low_;

    double velocity_, steering_, heading_;
    double trans_cov_, rot_cov_, eps_cov_;
    double ds_, dth_, dx_, dy_, x_, y_;
    bool tf_publish_;

};
}   