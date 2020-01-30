#include<ros/ros.h>
#include"random_number_generator/rn.h"
#include<ctime>
#include<cstdlib>

using namespace std;

int limitNum;

int main(int argc, char *argv[])
{

    ros::init(argc,argv,"rn_pub");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<random_number_generator::rn>("rn_info", 100);
    nh.setParam("limit_num", 10);
    random_number_generator::rn msg;

    srand(time(NULL));

    int randNum;

    ros::Rate loop_rate(10);
    while(ros::ok()){
        nh.getParam("limit_num", limitNum);
        ROS_INFO("limit param = %d", limitNum);

        if(limitNum <= 0) {
            ROS_ERROR("ERROR!!! parameter is smaller than 1...");
            break;
        }
        randNum = rand() % limitNum;
        msg.limit = limitNum;
        msg.randint = randNum;
        pub.publish(msg);
        loop_rate.sleep();
    }

    return 0;
}
