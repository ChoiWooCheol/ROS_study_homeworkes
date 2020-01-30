#include<ros/ros.h>
#include"marker_visualizer/marker_info.h"
#include<visualization_msgs/Marker.h>

class MarkerPublisher{
public:
    MarkerPublisher(){
        markerinfo_sub = nh.subscribe("marker_info", 10, &MarkerPublisher::markerInfoCB, this);
        rviz_pub = nh.advertise<visualization_msgs::Marker>("marker",10);
    }

    void markerInfoCB(const marker_visualizer::marker_infoConstPtr& ptr){
        shape = ptr->shape;
        x = ptr->x;
        y = ptr->y;
    }
    void genMsg(){
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.z = 1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        if(shape == "sphere")
            marker.type = visualization_msgs::Marker::SPHERE;
        else if(shape == "cube")
            marker.type = visualization_msgs::Marker::CUBE;
        else
            marker.type = visualization_msgs::Marker::ARROW;
        
        marker.pose.position.x = x;
        marker.pose.position.y = y;
    }
    void publish(){
        rviz_pub.publish(marker);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber markerinfo_sub;
    ros::Publisher rviz_pub;

    std::string shape;
    int x, y;
    visualization_msgs::Marker marker;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "marker_publisher");
    MarkerPublisher mp;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        mp.genMsg();
        mp.publish();
        loop_rate.sleep();
    }
    
    return 0;
}