#include <ros/ros.h>
#include <serial/serial.h>
#include <mutex>
#include <thread>
#include <deque>
#include <exception>
#include <ctime>
#include<cmath>
#include<cstdlib>
#include<vector>

#include "platform_rx_msg/platform_rx_msg.h"

serial::Serial ser;
std::mutex lock;
/* ---------------------------platform_rx.launch---------------------------- 
<launch>
    <node pkg="platform_rx_thread_test" name="rx_thread_test" type="rx_thread_test">
        <param name="path" value="/dev/ttyUSB0"/>
        <param name="serial_frequency" value="20"/>
        <param name="publish_frequency" value="20"/>
        <param name="moving_average_element_number" value="5"/>
    </node>
</launch>
------------------------------------------------------------------------------

path 값 오류 --> 터미널에서 dmesg | grep tty -> platform과 연결되어있는 USB포트를 바꿔주면됨
*/
#define ALIVE_UPDATE_FRE 21
#define EPSILON 0.1

class ParamReader{
public:
    ParamReader(ros::NodeHandle& nh){
        if(!nh.getParam("rx_thread_test/path", path)) throw std::runtime_error("give me path!");
        if(!nh.getParam("rx_thread_test/serial_frequency", serial_frequency)) 
            throw std::runtime_error("give me serial_frequency!");
        if(!nh.getParam("rx_thread_test/publish_frequency", publish_frequency)) 
            throw std::runtime_error("give me publish_frequency!");
        if(!nh.getParam("rx_thread_test/moving_average_element_number", moving_average_element_number))
            throw std::runtime_error("give me moving_average_element_number!");
        if(moving_average_element_number <= 1) std::runtime_error("too small element number...");
    }
    std::string path;
    int serial_frequency;
    int publish_frequency;
    int moving_average_element_number;
};

uint8_t packet[18] = {0};

template <typename T>
T getParsingData(const uint8_t *dataArray, int startIndex){//패킷이 문자열형태로 한번에 날라오는데 거기서 우리가필요한 속도값같은 것의 포트시작번지만큼 더해준다.
    T re_ = *(T*)(dataArray + startIndex);
    return re_;
}

void readSerial(int serial_read_loop_rate){
    ros::Rate loop_rate(serial_read_loop_rate);

    std::string raw;
    size_t error_cnt = 0;
    while(true){
        //read from serial
        if(ser.available() >= 18) raw = ser.read(ser.available());// available() = 버퍼에있는 숫자들의 characters을 리턴
        else loop_rate.sleep(); 

        //save in process variable
        const char *check = raw.c_str();
        if((check[0] == 0x53) & (check[1] == 0x54) & (check[2] == 0x58) & check[16] == 0x0D & check[17] == 0x0a ){
            lock.lock();
            for(int i = 0 ; i < 18; ++i) *(packet + i) = *(check + i);
            lock.unlock();
        }
        loop_rate.sleep();
    }
}

double SpFilter(double speed, double w, std::deque<double>& vec, int moveAvg){
    double standard;
    double sum = 0;
    for(int i = 0; i < moveAvg; i++){
        sum +=vec[i];
    }
    
    standard = sum / 5.0;
    
    double nextSpeed = standard + ((speed - standard) / w);
    return nextSpeed;
}

inline double calc_angle(const int16_t& st){
    double ang = st * 0.009624;
    return ang;
}

int main (int argc, char** argv){
    ros::init(argc, argv, "rx_thread_test");
    ros::NodeHandle nh;

    ParamReader reader(nh); //ParamReader 클래스 객체 생성, 생성자 파라미터로 노드핸들러 nh 선언.

    ros::Publisher pub = nh.advertise<platform_rx_msg::platform_rx_msg>("raw/platform_rx", 1000);

    try
    {
        ser.setPort(reader.path);//직렬포트의 식별자 설정 Parameter = string참조형태 
        ser.setBaudrate(115200);//직렬포트의 전송속도를 설정함. 115200
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); //Timeout 구조체 생성 Parameter = 읽기 또는 쓰기 호출이 발생한 후 시간 초과가 발생할 때까지의 시간
        ser.setTimeout(to);//Timeout 구조체를 사용해서 읽기쓰기 제한시간 설정 읽기또는쓰기가 완료되면 시간초과가 발생하거나 예외발생
        ser.open();//포트연결
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR("Unable to open port : %s",e.what());
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        ROS_ERROR("RX open error!");
        return -1;
    }

    std::thread thr(readSerial, reader.serial_frequency); //스레드 생성. 스레드 thr이 수행하는 함수는 readSerial(), 인자는 ParamReader객체 reader의
                                                            //serial_frequency임. // serial_frequency의 값이 뭔지 어떻게암? = launch파일에 20이라고 명시
                                                            //20 hz로 받아와야 적절. 원래는 50인데 50으로 딱맞추다보면 시리얼 통신 시작과 동시에 사람이 50헤르츠로 받을수 없어서
                                                            //20이 적당수치임.
    thr.detach(); //메인 스레드와 thr스레드 분리.

    platform_rx_msg::platform_rx_msg msg;
    ros::Rate loop_rate(reader.publish_frequency);
    size_t seq = 0;
    uint8_t packet_main[18] ={0};

    //speed calcuration member
    typedef int8_t ALIVE_datatype;
    std::deque<std::pair<int32_t, ALIVE_datatype> > encoder(reader.moving_average_element_number);//pair(a,b) a타입,b타입 각각의 변수를 가지는 변수타입.
                                                                                                //encoder는 deque형 객체인데 인덱스변수는 pair임(인덱스하나당 변수2개)


    int alive_gap_d = 0;
    int alive_gap_seconds = 0;
    const ros::Time init_alive_gap_time = ros::Time::now();
 //speed = (encoder[0].first - encoder[1].first) / encoderValuePerCycle * distanceValuePerCycle 
         //   / timeInterval / interval;

         //이 밑에 나오는 람다 함수는 속도를 계산하기위한 람다함수.
    auto calc_speed =[&]()->double{
        double total_encoder_gap = encoder.front().first - encoder.back().first; //encoder의 first= packet_main[11] front, back의 차이 = encoder의 변화값
        ALIVE_datatype alive_gap = encoder.front().second - encoder.back().second; //encoder의 second = packet_main[15] front, back의 차이 = alive의 변화값
        
        //ALIVE DEBUG 
        alive_gap_d += (int)alive_gap;
        alive_gap_seconds = (ros::Time::now() - init_alive_gap_time).sec;
        //ROS_INFO("gap: %d,  time: %d(sec), alive_hz : %.3lf(hz)", alive_gap_d, alive_gap_seconds, 
        //        (double)alive_gap_d/alive_gap_seconds);

        double time_interval = alive_gap * 1.0 / ALIVE_UPDATE_FRE; // platform send information for 50hz // ? * 1.0 ==> make double type, ALIVE_UPDATE_FRE = 21;
        double speed = total_encoder_gap / 99.2 * 1.655 / time_interval; //99.2상수 값 바꿔줘야함. 엔코더 / 돌린횟수(이론상 100이맞지만 기계가 오래돼서 오차발생)
                                                                        //1.655 = r*2*PI
        return speed;
    };

    //init
    for (int i = 0 ; i < 5; ++i){
        //get packet
        lock.lock();
        for(int i = 0 ; i < 18; ++i) *(packet_main + i) = *(packet + i);
        lock.unlock();

        //get serial sequence
        ALIVE_datatype alive = getParsingData<ALIVE_datatype>(packet_main, 15); // platform에서보내주는 serial packet[15]의 값이 자동차의 alive값임.
        
        encoder.push_front(std::make_pair(getParsingData<int32_t>(packet_main, 11), alive)); //encoder맨앞에 pair형 변수를 넣는다.
        encoder.pop_back();
        seq += abs((int)encoder.front().first - (encoder.begin() + 1)->first); //abs = 절대값(int)
        loop_rate.sleep();
    }

    std::deque<double> speedVec(reader.moving_average_element_number, 0.0);
    int moveCount = 0; // 속도 벡터가 평균을 만들려면 5개의 방이 다 차있어야함. moveCount가 최소 5이상
    double nextSP; // next speed by filter
    while(ros::ok()){
        //get packet
        lock.lock();
        for(int i = 0 ; i < 18; ++i) *(packet_main + i) = *(packet + i);
        lock.unlock();

        //get serial sequence
        ALIVE_datatype alive = getParsingData<ALIVE_datatype>(packet_main, 15);
        
        encoder.push_front(std::make_pair( getParsingData<int32_t>(packet_main, 11), alive ));
        encoder.pop_back();
        
        if (encoder[0].second == encoder[1].second) { //when encoder is not updated, continue;
            loop_rate.sleep();
            continue;
        }
        msg.speed = calc_speed();
       
        moveCount++;
        if(moveCount >= reader.moving_average_element_number && calc_speed() != 0.0){
            nextSP = SpFilter(calc_speed(), 3.3, speedVec, reader.moving_average_element_number);
           // ROS_INFO("speedfilter = %f", calc_speed());
            speedVec.push_front(nextSP);
            speedVec.pop_back();
            msg.speedfilter = nextSP;
        }
        else if(moveCount >= reader.moving_average_element_number && calc_speed () == 0.0){
           // ROS_INFO("speedfilter = %f", calc_speed());
            speedVec.push_front(0.0);
            speedVec.pop_back();
            nextSP = SpFilter(0.0, 3.3, speedVec, reader.moving_average_element_number);
            msg.speedfilter = nextSP;
        }

        msg.angle = calc_angle(getParsingData<int16_t>(packet_main, 8));
        msg.steer = getParsingData<int16_t>(packet_main, 8);
        msg.brake = getParsingData<uint8_t>(packet_main, 10);
        msg.seq = alive_gap_d;
        bool estop = getParsingData<uint8_t>(packet_main, 4);
        bool speed_flag = (fabs(msg.speed) < EPSILON) ? true : false;   //fabs = 절대값(double)
        nh.setParam("isEstop", getParsingData<uint8_t>(packet_main, 4));
        pub.publish(msg);     
        loop_rate.sleep();
    }
}
