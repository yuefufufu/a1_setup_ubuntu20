/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_REAL_ROBOT

#include "interface/IOSDK.h"
#include "interface/WirelessHandle.h"
#include "interface/KeyBoard.h"   /*追加*/
#include <stdio.h>
#include <tf/transform_broadcaster.h>

#ifdef ROBOT_TYPE_Go1
IOSDK::IOSDK():_safe(UNITREE_LEGGED_SDK::LeggedType::Aliengo), _udp(UNITREE_LEGGED_SDK::LOWLEVEL, 8090, "192.168.123.10", 8007){
    std::cout << "The control interface for real robot" << std::endl;
    _udp.InitCmdData(_lowCmd);
    cmdPanel = new KeyBoard();  /*変更*/
    //cmdPanel = new WirelessHandle();

#ifdef COMPILE_WITH_MOVE_BASE
    _pub = _nh.advertise<sensor_msgs::JointState>("/realRobot/joint_states", 20);
    _joint_state.name.resize(12);
    _joint_state.position.resize(12);
    _joint_state.velocity.resize(12);
    _joint_state.effort.resize(12);
#endif  // COMPILE_WITH_MOVE_BASE
}
#endif

#ifdef ROBOT_TYPE_A1
IOSDK::IOSDK():_safe(UNITREE_LEGGED_SDK::LeggedType::Aliengo), _udp(UNITREE_LEGGED_SDK::LOWLEVEL){
    std::cout << "The control interface for real robot" << std::endl;
    _udp.InitCmdData(_lowCmd);
    cmdPanel = new KeyBoard();  /*変更*/
    //cmdPanel = new WirelessHandle();

#ifdef COMPILE_WITH_MOVE_BASE
    _pub = _nh.advertise<sensor_msgs::JointState>("/realRobot/joint_states", 20);
    _joint_state.name.resize(12);
    _joint_state.position.resize(12);
    _joint_state.velocity.resize(12);
    _joint_state.effort.resize(12);
#endif  // COMPILE_WITH_MOVE_BASE
}
#endif


void IOSDK::sendRecv(const LowlevelCmd *cmd, LowlevelState *state){
    for(int i(0); i < 12; ++i){
        _lowCmd.motorCmd[i].mode = cmd->motorCmd[i].mode;
        _lowCmd.motorCmd[i].q    = cmd->motorCmd[i].q;
        _lowCmd.motorCmd[i].dq   = cmd->motorCmd[i].dq;
        _lowCmd.motorCmd[i].Kp   = cmd->motorCmd[i].Kp;
        _lowCmd.motorCmd[i].Kd   = cmd->motorCmd[i].Kd;
        _lowCmd.motorCmd[i].tau  = cmd->motorCmd[i].tau;
    }

    if(_lowCmd.motorCmd[2].q != 0){
        FRb_x = (-1 * 0.2 * sin(_lowCmd.motorCmd[1].q + _lowCmd.motorCmd[2].q)) + (-1 * 0.2 * sin(_lowCmd.motorCmd[1].q));
        FRb_y = (-1 * 0.2 * sin(_lowCmd.motorCmd[0].q) * cos(_lowCmd.motorCmd[1].q + _lowCmd.motorCmd[2].q)) + (-1 * 0.2 * sin(_lowCmd.motorCmd[0].q) * cos(_lowCmd.motorCmd[1].q)) - (0.0838 * cos(_lowCmd.motorCmd[0].q));
        FRb_z = (-1 * 0.2 * cos(_lowCmd.motorCmd[0].q) * cos(_lowCmd.motorCmd[1].q + _lowCmd.motorCmd[2].q)) + (-1 * 0.2 * cos(_lowCmd.motorCmd[0].q) * cos(_lowCmd.motorCmd[1].q)) + (0.0838 * sin(_lowCmd.motorCmd[0].q));
        FLb_x = (-1 * 0.2 * sin(_lowCmd.motorCmd[4].q + _lowCmd.motorCmd[5].q) + (-1 * 0.2 * sin(_lowCmd.motorCmd[4].q)));
        FLb_y = (-1 * 0.2 * sin(_lowCmd.motorCmd[3].q) * cos(_lowCmd.motorCmd[4].q + _lowCmd.motorCmd[5].q)) + (-1 * 0.2 * sin(_lowCmd.motorCmd[3].q) * cos(_lowCmd.motorCmd[4].q)) + (0.0838 * cos(_lowCmd.motorCmd[3].q));
        FLb_z = (-1 * 0.2 * cos(_lowCmd.motorCmd[3].q) * cos(_lowCmd.motorCmd[4].q + _lowCmd.motorCmd[5].q)) + (-1 * 0.2 * cos(_lowCmd.motorCmd[3].q) * cos(_lowCmd.motorCmd[4].q)) - (0.0838 * sin(_lowCmd.motorCmd[3].q));
        RRb_x = (-1 * 0.2 * sin(_lowCmd.motorCmd[7].q + _lowCmd.motorCmd[8].q) + (-1 * 0.2 * sin(_lowCmd.motorCmd[7].q)));
        RRb_y = (-1 * 0.2 * sin(_lowCmd.motorCmd[6].q) * cos(_lowCmd.motorCmd[7].q + _lowCmd.motorCmd[8].q)) + (-1 * 0.2 * sin(_lowCmd.motorCmd[6].q) * cos(_lowCmd.motorCmd[7].q)) - (0.0838 * cos(_lowCmd.motorCmd[6].q));
        RRb_z = (-1 * 0.2 * cos(_lowCmd.motorCmd[6].q) * cos(_lowCmd.motorCmd[7].q + _lowCmd.motorCmd[8].q)) + (-1 * 0.2 * cos(_lowCmd.motorCmd[6].q) * cos(_lowCmd.motorCmd[7].q)) + (0.0838 * sin(_lowCmd.motorCmd[6].q));
        RLb_x = (-1 * 0.2 * sin(_lowCmd.motorCmd[10].q + _lowCmd.motorCmd[11].q) + (-1 * 0.2 * sin(_lowCmd.motorCmd[10].q)));
        RLb_y = (-1 * 0.2 * sin(_lowCmd.motorCmd[9].q) * cos(_lowCmd.motorCmd[10].q + _lowCmd.motorCmd[11].q)) + (-1 * 0.2 * sin(_lowCmd.motorCmd[9].q) * cos(_lowCmd.motorCmd[10].q)) + (0.0838 * cos(_lowCmd.motorCmd[9].q));
        RLb_z = (-1 * 0.2 * cos(_lowCmd.motorCmd[9].q) * cos(_lowCmd.motorCmd[10].q + _lowCmd.motorCmd[11].q)) + (-1 * 0.2 * cos(_lowCmd.motorCmd[9].q) * cos(_lowCmd.motorCmd[10].q)) - (0.0838 * sin(_lowCmd.motorCmd[9].q));

        FR_z =  FRb_z - (2 * (L[0] + TL[1]) * tan(roll));
        FL_z =  FRb_z + (2 * (L[0] + TL[1]) * tan(roll));
        RR_z =  FRb_z - (2 * (L[0] + TL[1]) * tan(roll));
        RL_z =  FRb_z + (2 * (L[0] + TL[1]) * tan(roll));
        FR_z =  FR_z - (2 * TL[0] * tan(pitch));
        FL_z =  FR_z - (2 * TL[0] * tan(pitch));
        RR_z =  FR_z + (2 * TL[0] * tan(pitch));
        RL_z =  FR_z + (2 * TL[0] * tan(pitch));

/*        std::cout << "-----------------------------" << std::endl;
        std::cout << _lowCmd.motorCmd[0].q << "," << _lowCmd.motorCmd[1].q << "," << _lowCmd.motorCmd[2].q << std::endl;
        std::cout << FR_x << "," << FR_y << "," << FR_z << std::endl;
        std::cout << FRb_x << "," << FRb_y << "," << FRb_z << std::endl;
        std::cout << FRb_x - FR_x << "," << FRb_y - FR_y << "," << FRb_z - FR_z << std::endl;
*/
        L[3] = std::pow(FR_x, 2.0)+std::pow(FR_y, 2.0)+std::pow(FR_z, 2.0);
        _lowCmd.motorCmd[2].q = -acos((L[3]-std::pow(L[0], 2.0)-std::pow(L[1], 2.0)-std::pow(L[2], 2.0)) / (2*L[1]*L[2]));
        _lowCmd.motorCmd[1].q = -asin(FR_x / std::pow(L[3]-std::pow(L[0], 2.0), 0.5)) - atan((L[2]*sin(_lowCmd.motorCmd[2].q)) / (L[1]+(L[2]*cos(_lowCmd.motorCmd[2].q))));
        L[4] = (L[1]*cos(_lowCmd.motorCmd[1].q)) + (L[2]*cos(_lowCmd.motorCmd[1].q)*cos(_lowCmd.motorCmd[2].q)) - (L[2]*sin(_lowCmd.motorCmd[1].q)*sin(_lowCmd.motorCmd[2].q));
        _lowCmd.motorCmd[0].q = (-1 * asin(FR_y / std::pow(std::pow(L[0], 2.0)+std::pow(L[4], 2.0), 0.5))) - (atan(L[0]/L[4]));
        L[3] = std::pow(FL_x, 2.0)+std::pow(FL_y, 2.0)+std::pow(FL_z, 2.0);
        _lowCmd.motorCmd[5].q = -acos((L[3]-std::pow(L[0], 2.0)-std::pow(L[1], 2.0)-std::pow(L[2], 2.0)) / (2*L[1]*L[2]));
        _lowCmd.motorCmd[4].q = -asin(FL_x / std::pow(L[3]-std::pow(L[0], 2.0), 0.5)) - atan((L[2]*sin(_lowCmd.motorCmd[5].q)) / (L[1]+(L[2]*cos(_lowCmd.motorCmd[5].q))));
        L[4] = (L[1]*cos(_lowCmd.motorCmd[4].q)) + (L[2]*cos(_lowCmd.motorCmd[4].q)*cos(_lowCmd.motorCmd[5].q)) - (L[2]*sin(_lowCmd.motorCmd[4].q)*sin(_lowCmd.motorCmd[5].q));
        _lowCmd.motorCmd[3].q = (-1 * asin(FL_y / std::pow(std::pow(L[0], 2.0)+std::pow(L[4], 2.0), 0.5))) + (atan(L[0]/L[4]));
        L[3] = std::pow(RR_x, 2.0)+std::pow(RR_y, 2.0)+std::pow(RR_z, 2.0);
        _lowCmd.motorCmd[8].q = -acos((L[3]-std::pow(L[0], 2.0)-std::pow(L[1], 2.0)-std::pow(L[2], 2.0)) / (2*L[1]*L[2]));
        _lowCmd.motorCmd[7].q = -asin(RR_x / std::pow(L[3]-std::pow(L[0], 2.0), 0.5)) - atan((L[2]*sin(_lowCmd.motorCmd[8].q)) / (L[1]+(L[2]*cos(_lowCmd.motorCmd[8].q))));
        L[4] = (L[1]*cos(_lowCmd.motorCmd[7].q)) + (L[2]*cos(_lowCmd.motorCmd[7].q)*cos(_lowCmd.motorCmd[8].q)) - (L[2]*sin(_lowCmd.motorCmd[7].q)*sin(_lowCmd.motorCmd[8].q));
        _lowCmd.motorCmd[6].q = (-1 * asin(RR_y / std::pow(std::pow(L[0], 2.0)+std::pow(L[4], 2.0), 0.5))) - (atan(L[0]/L[4]));
        L[3] = std::pow(RL_x, 2.0)+std::pow(RL_y, 2.0)+std::pow(RL_z, 2.0);
        _lowCmd.motorCmd[11].q = -acos((L[3]-std::pow(L[0], 2.0)-std::pow(L[1], 2.0)-std::pow(L[2], 2.0)) / (2*L[1]*L[2]));
        _lowCmd.motorCmd[10].q = -asin(RL_x / std::pow(L[3]-std::pow(L[0], 2.0), 0.5)) - atan((L[2]*sin(_lowCmd.motorCmd[11].q)) / (L[1]+(L[2]*cos(_lowCmd.motorCmd[11].q))));
        L[4] = (L[1]*cos(_lowCmd.motorCmd[10].q)) + (L[2]*cos(_lowCmd.motorCmd[10].q)*cos(_lowCmd.motorCmd[11].q)) - (L[2]*sin(_lowCmd.motorCmd[10].q)*sin(_lowCmd.motorCmd[11].q));
        _lowCmd.motorCmd[9].q = (-1 * asin(RL_y / std::pow(std::pow(L[0], 2.0)+std::pow(L[4], 2.0), 0.5))) + (atan(L[0]/L[4]));
    }

    if(_lowCmd.motorCmd[0].q < -0.45){
        _lowCmd.motorCmd[0].q == -0.45;}
    if(_lowCmd.motorCmd[3].q > 0.45){
        _lowCmd.motorCmd[3].q == 0.45;}
    if(_lowCmd.motorCmd[6].q < -0.45){
        _lowCmd.motorCmd[6].q == -0.45;}
    if(_lowCmd.motorCmd[9].q > 0.45){
        _lowCmd.motorCmd[9].q == 0.45;}

    _udp.SetSend(_lowCmd);
    _udp.Send();

    _udp.Recv();
    _udp.GetRecv(_lowState);

    for(int i(0); i < 12; ++i){
        state->motorState[i].q = _lowState.motorState[i].q;
        state->motorState[i].dq = _lowState.motorState[i].dq;
        state->motorState[i].ddq = _lowState.motorState[i].ddq;
        state->motorState[i].tauEst = _lowState.motorState[i].tauEst;
        state->motorState[i].mode = _lowState.motorState[i].mode;
    }

    for(int i(0); i < 3; ++i){
        state->imu.quaternion[i] = _lowState.imu.quaternion[i];
        state->imu.gyroscope[i]  = _lowState.imu.gyroscope[i];
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
    }
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];
    tf::Quaternion quat(state->imu.quaternion[1],state->imu.quaternion[2],state->imu.quaternion[3],state->imu.quaternion[0]);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    cmdPanel->receiveHandle(&_lowState);
    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();

#ifdef COMPILE_WITH_MOVE_BASE
    _joint_state.header.stamp = ros::Time::now();
    _joint_state.name = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", 
                         "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",  
                         "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", 
                         "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
    for(int i(0); i<12; ++i){
        _joint_state.position[i] = state->motorState[i].q;
        _joint_state.velocity[i] = state->motorState[i].dq;
        _joint_state.effort[i]   = state->motorState[i].tauEst;
    }

    _pub.publish(_joint_state);
#endif  // COMPILE_WITH_MOVE_BASE
}

#endif  // COMPILE_WITH_REAL_ROBOT