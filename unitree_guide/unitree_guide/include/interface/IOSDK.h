/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef IOSDK_H
#define IOSDK_H

#include "interface/IOInterface.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#ifdef COMPILE_WITH_MOVE_BASE
    #include <ros/ros.h>
    #include <ros/time.h>
    #include <sensor_msgs/JointState.h>
#endif  // COMPILE_WITH_MOVE_BASE


class IOSDK : public IOInterface{
public:
IOSDK();
~IOSDK(){}
void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

private:
double roll = 0;
double pitch = 0;
double yaw = 0;
double FR_x = 0;
double FR_y = 0;
double FR_z = 0;
double FL_x = 0;
double FL_y = 0;
double FL_z = 0;
double RR_x = 0;
double RR_y = 0;
double RR_z = 0;
double RL_x = 0;
double RL_y = 0;
double RL_z = 0;
double FRb_x = 0;
double FRb_y = 0;
double FRb_z = 0;
double FLb_x = 0;
double FLb_y = 0;
double FLb_z = 0;
double RRb_x = 0;
double RRb_y = 0;
double RRb_z = 0;
double RLb_x = 0;
double RLb_y = 0;
double RLb_z = 0;
double L[5] = {0.0838, 0.2, 0.2, 0, 0};
double TL[3] = {0.1805, 0.047, 0};

UNITREE_LEGGED_SDK::UDP _udp;
UNITREE_LEGGED_SDK::Safety _safe;
UNITREE_LEGGED_SDK::LowCmd _lowCmd;
UNITREE_LEGGED_SDK::LowState _lowState;

#ifdef COMPILE_WITH_MOVE_BASE
    ros::NodeHandle _nh;
    ros::Publisher _pub;
    sensor_msgs::JointState _joint_state;
#endif  // COMPILE_WITH_MOVE_BASE
};

#endif  // IOSDK_H