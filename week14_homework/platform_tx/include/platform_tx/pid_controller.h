#include <ros/ros.h>
#include <ros/time.h>

class PID_Controller {
public:
PID_Controller()
    : current_speed_(0.0), current_steer_(0.0)
    , ref_speed_(0.0), ref_steer_(0.0)
    , err_speed_(0.0), err_steer_(0.0)
    , cmd_speed_(0.0), cmd_steer_(0.0), cmd_brake_(0.0)
    , kp_speed_(0.0), ki_speed_(0.0), kd_speed_(0.0)
    , kp_steer_(0.0), ki_steer_(0.0), kd_steer_(0.0)
    , kp_brake_(0.0), ki_brake_(0.0), kd_brake_(0.0)
{
    priv_nh_.param<double>("/control/speed/kp",kp_speed_, 0.0);
    priv_nh_.param<double>("/control/speed/ki",ki_speed_, 0.0);
    priv_nh_.param<double>("/control/speed/kd",kd_speed_, 0.0); 

    priv_nh_.param<double>("/control/steer/kp",kp_steer_, 0.0);
    priv_nh_.param<double>("/control/steer/ki",ki_steer_, 0.0);
    priv_nh_.param<double>("/control/steer/kd",kd_steer_, 0.0);
    
    priv_nh_.param<double>("/control/brake/kp",kp_brake_, 0.0);
    priv_nh_.param<double>("/control/brake/ki",ki_brake_, 0.0);
    priv_nh_.param<double>("/control/brake/kd",kd_brake_, 0.0);     
}

void Init(void) // Controller 돌리기 전에 initialize (dt 계산을 위한 time 초기값)
{
    timestamp_ = ros::Time::now();
}

void Read_State(double speed, double steer) // "Platform_RX 받는 부분에 사용 (Platform Unit-> SI Unit으로 변환해서 넣기)
{
    current_speed_ = speed;
    current_steer_ = steer;
}

void Read_Reference(double speed, double steer) // "ackermann_cmd 받는 부분에 사용 (Platform Unit-> SI Unit으로 변환해서 넣기)
{
    ref_speed_ = speed;
    ref_steer_ = steer;
}

void UpdateParameters(void){
     priv_nh_.getParam("/control/speed/kp", kp_speed_);
     priv_nh_.getParam("/control/speed/ki", ki_speed_);
     priv_nh_.getParam("/control/speed/kd", kd_speed_);

     priv_nh_.getParam("/control/steer/kp", kp_steer_);
     priv_nh_.getParam("/control/steer/ki", ki_steer_);
     priv_nh_.getParam("/control/steer/kd", kd_steer_);
     
     priv_nh_.getParam("/control/brake/kp", kp_brake_);
     priv_nh_.getParam("/control/brake/ki", ki_brake_);
     priv_nh_.getParam("/control/brake/kd", kd_brake_);
}
 
void Calc_PID(void) // read_state, read_reference로 읽은 후에 Platform_TX(SI Unit -> Platform Unit) 하기 전에 호출
{    
    ros::Time now = ros::Time::now();
    const double dt = (now - timestamp_).toSec();
    timestamp_ = now;

    UpdateParameters(); // PID Gain Parameter Update (PID_Controller::UpdateParameters();)
    
    // SPEED CONTROL
    err_speed_ = ref_speed_ - current_speed_;
    if(err_speed_ >= 0.0){
        cmd_speed_ = err_speed_ * (kp_speed_ + ki_speed_*dt + kd_speed_/dt);
        cmd_brake_ = 0.0;
    }
    else{
        cmd_speed_ = ref_speed_;
        cmd_brake_ = (-err_speed_) * (kp_brake_ + ki_brake_*dt + kd_brake_/dt);
    }
    
    // STEER CONTROL
    err_steer_ = ref_steer_ - current_steer_;
    cmd_steer_ = err_steer_ * (kp_steer_ + ki_steer_*dt + kd_steer_/dt);
}

inline double cmd_speed(void){ return cmd_speed_; }
inline double cmd_steer(void){ return cmd_steer_; }
inline double cmd_brake(void){ return cmd_brake_; }

private:
    ros::NodeHandle priv_nh_;
    ros::Time timestamp_;
  
    double current_speed_, current_steer_;
    double ref_speed_, ref_steer_;
    double err_speed_, err_steer_;
    double cmd_speed_, cmd_steer_, cmd_brake_;
    
    double kp_speed_, ki_speed_, kd_speed_;
    double kp_steer_, ki_steer_, kd_steer_;
    double kp_brake_, ki_brake_, kd_brake_;
};

