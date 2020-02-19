# ROS_study_homeworkes
숭실대학교 ROS 스터디를 진행하면서 스터디를 듣는 학생들에게 매주마다 내주는 과제입니다.

과제는 ROS_wiki를 인용한 것도 있으며, 직접 만든 코드들도 있습니다.

# Homework information
 * WEEK1_HOMEWORK
    * ROS의 기본적인 publisher와 subscriber를 해석하고 이해하는 코드를 과제로 내주었습니다.
    * 사용자 정의 메세지를 작성하는 방법을 수업하였습니다.
    * ROS의 기본적 틀을 이해하는 코드를 작성하도록 하였습니다.
    
 * WEEK2_HOMEWORK
    * 1주차에 배운 publisher와 subscriber로 숫자를 메세지로 보내는 과제입니다.
    * 노드에 여러 publisher와 subscriber가 있을 수 있다는 것을 이해하게 하기위한 과제입니다.

 * WEEK3_HOMEWORK
    * ROS에서 제공하는 turtlebot 패키지의 내용을 숙지하기 위한 과제입니다.
    * 사용자가 정의하는 패키지, 메세지 말고 ROS의 표준 메세지도 있다는 것을 알기위한 과제입니다.
    * rostopic 명령어, rosnode 명령어, roscd 명령어를 숙지합니다.
   
 * WEEK4_HOMEWORK
    * 로봇을 다룰 때 필요한 tf 패키지를 이해하기 위한 과제입니다.
    * static tf, tf를 이해하고, rviz상에서 이해하는 과제입니다.
    
 * WEEK5_HOMEWORK
    * 선형대수 라이브러리 eigen을 이해하고 로봇공학에서 중요한 좌표계변환을 이해하기 위한 코드 입니다.
    * roll, pitch, yaw 변환을 이해합니다.
    * rotation, transform 행렬들을 이해합니다.
    
 * WEEK6_HOMEWORK
      * visualization_msgs/Marker 메세지를 이해하고, 이용하여 표준메세지를 publish하는 과제입니다.
      * rivz를 통해 maker를 확인합니다.
      
 * WEEK7_HOMEWORK
      * ROS의 파라미터를 이해하게 하기 위한 과제입니다.
      * serviceClient, serviceServer를 이용하여 코드를 작성하고, 변수를 파라미터로 받아옵니다.
      * getParam() 함수를 이해하도록 하였습니다.
      
 * WEEK9_HOMEWORK
      * gazebo 물리 시뮬레이터를 사용하여 mobile robot의 동작을 이해합니다.
      * 로봇에 대한 바퀴정보를 얻어와서 각 바퀴의 속도정보로 바꿉니다.
      * gazebo상에 로봇의 정보를 서브스크라이브 하여 비워져있는 함수가 동작하도록 합니다.
      
 * WEEK10_HOMEWORK
      * week9에서 배웠던 속도 계산을 이용하여 odometry정보로 변환하는 과제입니다.
      * 차량의 구조와, 각 바퀴들의 속도, 바퀴의 반지름 등의 정보를 이용하는 법을 학습합니다.
      * tf를 배운 것을 활용하여 rviz상에서 odometry정보를 확인 할 수 있도록 하는 과제입니다.

 * WEEK11_HOMEWORK
      * week10 homework의 내용과 비슷한 odometry정보를 만들어내는 코드를 분석하는 과제입니다.
      * teleop twist keyboard 노드에서 발행되는 /cmd_vel 토픽을 분석합니다.
      * /cmd_vel 토픽을 mobile robot에 적용하여 키보드 값에 따른 로봇의 위치를 계산합니다.
      * SLAM을 이해하기 위한 논문 분석을 진행하여 자율주행 mobile robot의 이해를 학습합니다.
      
 * WEEK12_HOMEWORK
      * amcl을 이해하고, 기초적인 자율주행 시스템을 이해합니다.
      * 시스템을 동작시키는데 필요한 노드나 launch 파일들을 한번에 구동시키는 별도의 launch 파일을 작성합니다.
      * navigation stack, move_base, gmapping을 이해합니다.
      * local path planning, global path planning의 기초적인 개념을 이해합니다.
      
* WEEK13_HOMEWORK
      * slam 논문 분석을 통해 자율주행 시스템에서 slam 알고리즘의 중요성을 파악할 수 있도록하는 과제입니다.
      * 왜 몬테카를로 방식을 사용하는지, local, global planning이 나눠져있는지 학습합니다.
      * ROS와 자율주행시스템을 이해한 후, 실제 ERP-42 언맨드솔루션 자동차 플랫폼에서 사용했던 odometry 패키지를 분석합니다.
