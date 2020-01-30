#include<ros/ros.h>
#include<eigen3/Eigen/Dense>
#include<iostream>

using namespace std;
using namespace Eigen;

Matrix4d RotZ(double rad){
    Matrix4d m;
    m <<    cos(rad),-sin(rad),0,0,
            sin(rad),cos(rad),0,0,
            0,0,1,0,
            0,0,0,1;
    return m;
}

Matrix4d TransZ(double d){
    Matrix4d m2;
    m2 <<   1,0,0,0,
            0,1,0,0,
            0,0,1,d,
            0,0,0,1;

    return m2;
}

Matrix4d TransX(double a){
    Matrix4d m2;
        m2 <<   1,0,0,a,
            0,1,0,0,
            0,0,1,0,
            0,0,0,1;
    return m2;
}

Matrix4d RotX(double rad){
    Matrix4d m3;
    m3 <<   1,0,0,0,
            0,cos(rad),-sin(rad),0,
            0,sin(rad),cos(rad),0,
            0,0,0,1;

    return m3;
}

int main(int argc, char *argv[])
{
    double PI = 3.14;
    ros::init(argc, argv, "example");
    ros::NodeHandle nh;
    Matrix4d m = RotZ(1) * TransZ(1) * TransX(0) * RotX(PI/2.0);

    cout << m << endl;
    return 0;
}
