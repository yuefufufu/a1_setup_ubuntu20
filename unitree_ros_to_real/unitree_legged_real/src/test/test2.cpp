#include <ros/ros.h>
#include <iostream>
#include <pthread.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "convert.h"
#include <geometry_msgs/Twist.h>
using namespace UNITREE_LEGGED_SDK;

double forwardS;
double sideS;
double rotateS;
double bodyHeight;

template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(1000);
    }
}

void cmd_callback(const geometry_msgs::Twist& msg_cmd)
{
    forwardS = msg_cmd.linear.x;
    sideS = msg_cmd.linear.y;
    bodyHeight = msg_cmd.linear.z;
    rotateS = msg_cmd.angular.z;
    //std::cout << forwardS*0.1 <<"/"<< sideS*0.1 <<"/"<< rotateS*0.1 << std::endl;
    ROS_INFO("%f,%f,%f",forwardS*0.1,sideS*0.1,rotateS*0.1);
}

template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);
    long motiontime = 0;
    TCmd SendHighLCM = {0};
    TState RecvHighLCM = {0};
    unitree_legged_msgs::HighCmd SendHighROS;
    unitree_legged_msgs::HighState RecvHighROS;
    roslcm.SubscribeState();
    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);
    ros::Subscriber cmd_sub = nh.subscribe("cmd_vel", 10, cmd_callback);

    while (ros::ok()){
        roslcm.Get(RecvHighLCM);
        RecvHighROS = ToRos(RecvHighLCM);
        SendHighROS.forwardSpeed = 0.0f;
        SendHighROS.sideSpeed = 0.0f;
        SendHighROS.rotateSpeed = 0.0f;
        SendHighROS.bodyHeight = 0.0f;
        SendHighROS.mode = 0;
        SendHighROS.roll  = 0;
        SendHighROS.pitch = 0;
        SendHighROS.yaw = 0;

        if(forwardS==0 && sideS==0 && rotateS==0){
            SendHighROS.mode = 1;}//1にしてみる？おすわり、電源切る、謎改行、ジャンプ
        else{
            SendHighROS.mode = 2;}

        if(forwardS>=0){
            SendHighROS.forwardSpeed = forwardS*0.1f;}
        else{
            SendHighROS.forwardSpeed = forwardS*0.3f;}

        SendHighROS.bodyHeight = bodyHeight*0.1f;
        SendHighROS.sideSpeed = sideS*0.3f;
        SendHighROS.rotateSpeed = rotateS*0.1f;
        
        SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
        roslcm.Send(SendHighLCM);
        ros::spinOnce();
        loop_rate.sleep(); 
    }
    return 0;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "walk_ros_mode");
    std::string firmwork;
    ros::param::get("/firmwork", firmwork);
    std::string robot_name;
    UNITREE_LEGGED_SDK::LeggedType rname;
    ros::param::get("/robot_name", robot_name);
    rname = UNITREE_LEGGED_SDK::LeggedType::A1;
    UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
    mainHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
}