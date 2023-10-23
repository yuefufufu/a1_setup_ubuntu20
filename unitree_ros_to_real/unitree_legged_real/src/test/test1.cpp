/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/
#include <ros/ros.h>
#include <pthread.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "convert.h"
#include <geometry_msgs/Twist.h>

#ifdef SDK3_1
using namespace aliengo;
#endif
#ifdef SDK3_2
using namespace UNITREE_LEGGED_SDK;
#endif

double forwardS;
double sideS;
double rotateS;

template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

void cmd_callback(const geometry_msgs::Twist& msg_cmd)
{
    forwardS = msg_cmd.linear.x;
    sideS = msg_cmd.linear.y;
    rotateS = msg_cmd.angular.z;
    std::cout << "unko" << std::endl;
}

template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle nh;
    ros::Rate loop_rate(500);

    // SetLevel(HIGHLEVEL);
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
        SendHighROS.mode = 2;

    std::cout << forwardS <<"/"<< sideS <<"/"<< rotateS << std::endl;

        SendHighROS.forwardSpeed = forwardS*0.1f;
        SendHighROS.sideSpeed = sideS*0.1f;
        SendHighROS.rotateSpeed = rotateS*0.1f;

/*
        if(motionbit[0]==1){
            SendHighROS.mode = 2;
            SendHighROS.forwardSpeed = 0.2f;
        }
        if(motionbit[1]==1){
            SendHighROS.mode = 2;
            SendHighROS.forwardSpeed = -0.2f;
        }
        if(motionbit[2]==1){
            SendHighROS.mode = 2;
            SendHighROS.sideSpeed = 0.2f;
        }
        if(motionbit[3]==1){
            SendHighROS.mode = 2;
            SendHighROS.sideSpeed = -0.2f;
        }
        if(motionbit[4]==1){
            SendHighROS.mode = 2;
            SendHighROS.rotateSpeed = 0.2f;
        }
        if(motionbit[5]==1){
            SendHighROS.mode = 2;
            SendHighROS.rotateSpeed = -0.2f;
        }
        if(motionbit[6]==1){
            std::cout << "6" << std::endl;
        }
        if(motionbit[7]==1){
            std::cout << "7" << std::endl;
        }
        if(motionbit[8]==1){
            std::cout << "8" << std::endl;
        }
*/

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

    #ifdef SDK3_1
        aliengo::Control control(aliengo::HIGHLEVEL);
        aliengo::LCM roslcm;
        mainHelper<aliengo::HighCmd, aliengo::HighState, aliengo::LCM>(argc, argv, roslcm);
    #endif

    #ifdef SDK3_2
        std::string robot_name;
        UNITREE_LEGGED_SDK::LeggedType rname;
        ros::param::get("/robot_name", robot_name);
        if(strcasecmp(robot_name.c_str(), "A1") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::A1;
        else if(strcasecmp(robot_name.c_str(), "Aliengo") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::Aliengo;

        // UNITREE_LEGGED_SDK::InitEnvironment();
        UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
        mainHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
    #endif
}