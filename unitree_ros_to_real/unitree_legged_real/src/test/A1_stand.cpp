/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <string>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "convert.h"

#ifdef SDK3_1
using namespace aliengo;
#endif
#ifdef SDK3_2
using namespace UNITREE_LEGGED_SDK;
#endif


double r0;
double r1;
double r2;
double d;
double h;
double theta[12];
double Hz = 1;

template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle n;
    ros::Rate loop_rate(500);

    long motiontime = 0;
    int rate_count = 0;
    int sin_count = 0;
    float F_qDes[3]={0};
    float R_qDes[3]={0};
    float F_q[3] = {0.0, 0.6, -1.25};
    float R_q[3] = {0.0, 0.8, -1.25};
    float Kp[3] = {13.0,13.0,25.0};  
    float Kd[3] = {4.0,4.0,5.0};
    TCmd SendLowLCM = {0};
    TState RecvLowLCM = {0};
    unitree_legged_msgs::LowCmd SendLowROS;
    unitree_legged_msgs::LowState RecvLowROS;

    bool initiated_flag = false;  // initiate need time
    int count = 0;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    SendLowROS.levelFlag = LOWLEVEL;
    for(int i = 0; i<12; i++){
        SendLowROS.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
        SendLowROS.motorCmd[i].q = PosStopF;        // 禁止位置环
        SendLowROS.motorCmd[i].Kp = 0;
        SendLowROS.motorCmd[i].dq = VelStopF;        // 禁止速度环
        SendLowROS.motorCmd[i].Kd = 0;
        SendLowROS.motorCmd[i].tau = 0;
    }

    while (ros::ok()){
        roslcm.Get(RecvLowLCM);
        RecvLowROS = ToRos(RecvLowLCM);

        ROS_INFO("----------------------");
        std::cout << std::fixed;
        std::cout << "FR_0:" << std::setprecision(5) << RecvLowROS.motorState[FR_0].q << ", FR_1:" << std::setprecision(5) << RecvLowROS.motorState[FR_1].q<< ", FR_2:" << std::setprecision(5) << RecvLowROS.motorState[FR_2].q << std::endl;
        std::cout << "FL_0:" << std::setprecision(5) << RecvLowROS.motorState[FL_0].q << ", FL_1:" << std::setprecision(5) << RecvLowROS.motorState[FL_1].q<< ", FL_2:" << std::setprecision(5) << RecvLowROS.motorState[FL_2].q << std::endl;
        std::cout << "RR_0:" << std::setprecision(5) << RecvLowROS.motorState[RR_0].q << ", RR_1:" << std::setprecision(5) << RecvLowROS.motorState[RR_1].q<< ", RR_2:" << std::setprecision(5) << RecvLowROS.motorState[RR_2].q << std::endl;
        std::cout << "RL_0:" << std::setprecision(5) << RecvLowROS.motorState[RL_0].q << ", RL_1:" << std::setprecision(5) << RecvLowROS.motorState[RL_1].q<< ", RL_2:" << std::setprecision(5) << RecvLowROS.motorState[RL_2].q << std::endl;

        motiontime++;
    {
        SendLowROS.motorCmd[FR_0].tau = -0.65f;
        SendLowROS.motorCmd[FL_0].tau = +0.65f;
        SendLowROS.motorCmd[RR_0].tau = -0.65f;
        SendLowROS.motorCmd[RL_0].tau = +0.65f;

        if( motiontime >= 0 && motiontime < 10){
            F_qDes[0] = F_q[0];
            F_qDes[1] = F_q[1];
            F_qDes[2] = F_q[2];
            R_qDes[0] = R_q[0];
            R_qDes[1] = R_q[1];
            R_qDes[2] = R_q[2];
        }

        if( motiontime >= 1000){
            sin_count++;
            F_qDes[0] = F_q[0];
            F_qDes[1] = F_q[1] -0.25 * sin( Hz * M_PI*sin_count/1000.0);
            F_qDes[2] = F_q[2] +0.5 * sin( Hz * M_PI*sin_count/1000.0);
            R_qDes[0] = R_q[0];
            R_qDes[1] = R_q[1] -0.25 * sin( Hz * M_PI*sin_count/1000.0);
            R_qDes[2] = R_q[2] +0.5 * sin( Hz * M_PI*sin_count/1000.0);
        }

        //FR
        {
        SendLowROS.motorCmd[FR_0].q = F_qDes[0];
        SendLowROS.motorCmd[FR_0].dq = 0;
        SendLowROS.motorCmd[FR_0].Kp = Kp[0];
        SendLowROS.motorCmd[FR_0].Kd = Kd[0];
        SendLowROS.motorCmd[FR_0].tau = -0.65f;

        SendLowROS.motorCmd[FR_1].q = F_qDes[1];
        SendLowROS.motorCmd[FR_1].dq = 0;
        SendLowROS.motorCmd[FR_1].Kp = Kp[1];
        SendLowROS.motorCmd[FR_1].Kd = Kd[1];
        SendLowROS.motorCmd[FR_1].tau = 0.0f;

        SendLowROS.motorCmd[FR_2].q =  F_qDes[2];
        SendLowROS.motorCmd[FR_2].dq = 0;
        SendLowROS.motorCmd[FR_2].Kp = Kp[2];
        SendLowROS.motorCmd[FR_2].Kd = Kd[2];
        SendLowROS.motorCmd[FR_2].tau = 0.0f;}
        //FL
        {
        SendLowROS.motorCmd[FL_0].q = F_qDes[0];
        SendLowROS.motorCmd[FL_0].dq = 0;
        SendLowROS.motorCmd[FL_0].Kp = Kp[0];
        SendLowROS.motorCmd[FL_0].Kd = Kd[0];
        SendLowROS.motorCmd[FL_0].tau = +0.65f;

        SendLowROS.motorCmd[FL_1].q = F_qDes[1];
        SendLowROS.motorCmd[FL_1].dq = 0;
        SendLowROS.motorCmd[FL_1].Kp = Kp[1];
        SendLowROS.motorCmd[FL_1].Kd = Kd[1];
        SendLowROS.motorCmd[FL_1].tau = 0.0f;

        SendLowROS.motorCmd[FL_2].q =  F_qDes[2];
        SendLowROS.motorCmd[FL_2].dq = 0;
        SendLowROS.motorCmd[FL_2].Kp = Kp[2];
        SendLowROS.motorCmd[FL_2].Kd = Kd[2];
        SendLowROS.motorCmd[FL_2].tau = 0.0f;}
        //RR
        {
        SendLowROS.motorCmd[RR_0].q = R_qDes[0];
        SendLowROS.motorCmd[RR_0].dq = 0;
        SendLowROS.motorCmd[RR_0].Kp = Kp[0];
        SendLowROS.motorCmd[RR_0].Kd = Kd[0];
        SendLowROS.motorCmd[RR_0].tau = -0.65f;

        SendLowROS.motorCmd[RR_1].q = R_qDes[1];
        SendLowROS.motorCmd[RR_1].dq = 0;
        SendLowROS.motorCmd[RR_1].Kp = Kp[1];
        SendLowROS.motorCmd[RR_1].Kd = Kd[1];
        SendLowROS.motorCmd[RR_1].tau = 0.0f;

        SendLowROS.motorCmd[RR_2].q =  R_qDes[2];
        SendLowROS.motorCmd[RR_2].dq = 0;
        SendLowROS.motorCmd[RR_2].Kp = Kp[2];
        SendLowROS.motorCmd[RR_2].Kd = Kd[2];
        SendLowROS.motorCmd[RR_2].tau = 0.0f;}
        //RL
        {
        SendLowROS.motorCmd[RL_0].q = R_qDes[0];
        SendLowROS.motorCmd[RL_0].dq = 0;
        SendLowROS.motorCmd[RL_0].Kp = Kp[0];
        SendLowROS.motorCmd[RL_0].Kd = Kd[0];
        SendLowROS.motorCmd[RL_0].tau = +0.65f;

        SendLowROS.motorCmd[RL_1].q = R_qDes[1];
        SendLowROS.motorCmd[RL_1].dq = 0;
        SendLowROS.motorCmd[RL_1].Kp = Kp[1];
        SendLowROS.motorCmd[RL_1].Kd = Kd[1];
        SendLowROS.motorCmd[RL_1].tau = 0.0f;

        SendLowROS.motorCmd[RL_2].q =  R_qDes[2];
        SendLowROS.motorCmd[RL_2].dq = 0;
        SendLowROS.motorCmd[RL_2].Kp = Kp[2];
        SendLowROS.motorCmd[RL_2].Kd = Kd[2];
        SendLowROS.motorCmd[RL_2].tau = 0.0f;}
    }

        SendLowLCM = ToLcm(SendLowROS, SendLowLCM);
        roslcm.Send(SendLowLCM);
        ros::spinOnce();
        loop_rate.sleep();

    }

    ROS_INFO("--------asdadsad-----");
    return 0;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "position_ros_mode");
    std::string firmwork;
    ros::param::get("/firmwork", firmwork);

    #ifdef SDK3_1
        aliengo::Control control(aliengo::LOWLEVEL);
        aliengo::LCM roslcm;
        mainHelper<aliengo::LowCmd, aliengo::LowState, aliengo::LCM>(argc, argv, roslcm);
    #endif

    #ifdef SDK3_2
        std::string robot_name;
        UNITREE_LEGGED_SDK::LeggedType rname;
        ros::param::get("/robot_name", robot_name);
        if(strcasecmp(robot_name.c_str(), "A1") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::A1;
        else if(strcasecmp(robot_name.c_str(), "Aliengo") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::Aliengo;
            
        // UNITREE_LEGGED_SDK::Control control(rname, UNITREE_LEGGED_SDK::LOWLEVEL);
        // UNITREE_LEGGED_SDK::InitEnvironment();
        UNITREE_LEGGED_SDK::LCM roslcm(LOWLEVEL);
        mainHelper<UNITREE_LEGGED_SDK::LowCmd, UNITREE_LEGGED_SDK::LowState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
    #endif
}