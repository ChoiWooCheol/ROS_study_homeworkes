#include<ros/ros.h>
#include"parameter_tutorials/SrvTutorial.h"

#define PLUS 1
#define MINUS 2
#define MULTIPLICATION 3
#define DIVISION 4


class SimpleCalculator{
public:
    int param;
    SimpleCalculator(std::string name_, std::string paramName_) : name(name_), paramName(paramName_){
        this->setParameter();
        param_server_ = nh.advertiseService(name.c_str(), &SimpleCalculator::calculation, this);
    }

    ~SimpleCalculator(){}
    void setParameter(){ nh.setParam(paramName.c_str(), PLUS); }

    bool calculation(parameter_tutorials::SrvTutorialRequest& req, parameter_tutorials::SrvTutorialResponse& res){
        nh.getParam(paramName.c_str(), param);
        
        switch(param){
           case PLUS:
                res.result = req.a + req.b; break;
           case MINUS:
                res.result = req.a - req.b; break;
           case MULTIPLICATION:
                res.result = req.a * req.b; break;
           case DIVISION:
                if(req.b == 0){ res.result = 0; break; }
                else{ res.result = req.a / req.b; break; }
           default:
                res.result = req.a + req.b; break;
        }

    ROS_INFO("result: x=%d, y=%d", (long int)req.a,(long int)req.b );
    ROS_INFO("sending back Response: [%d]", (long int)res.result );
    return true;
    }

public:
    ros::ServiceServer param_server_;
    ros::NodeHandle nh;
    std::string name;
    std::string paramName;
};

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"param_server");
    SimpleCalculator paramserver("parameter_tutorials", "calc_param");
    ros::spin();
        
    return 0;
}