#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "convert.h"
#include <tf/transform_broadcaster.h>
using namespace std;
using namespace UNITREE_LEGGED_SDK;

long cccount = 0;
double lin_x = 0;
double lin_y = 0;
double Hz[7] = {0, 0, 0, 0, 0, 0, 3000};//{4の倍数, 0, 0, 立ち上がり時間}
double flag = 0;
double c_flag = 0;
double hoge = 0;
double ang_z = 0;
unitree_legged_msgs::LowState RecvLowROS;
unitree_legged_msgs::LowCmd SendLowROS;
LowState state = {0};
LowCmd cmd = {0};


class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::A1){}
    void RobotSafe();

    Safety safe;
};

void Custom::RobotSafe()
{
    state.levelFlag = (uint8_t)RecvLowROS.levelFlag;
    state.commVersion = (uint16_t)RecvLowROS.commVersion;
    state.robotID = (uint16_t)RecvLowROS.robotID;
    state.SN = (uint32_t)RecvLowROS.SN;
    state.bandWidth = (uint8_t)RecvLowROS.bandWidth;
    for(int i=0;i<20;i++){
        state.motorState[i].mode = (uint8_t)RecvLowROS.motorState[i].mode;
        state.motorState[i].q = (float)RecvLowROS.motorState[i].q;
        state.motorState[i].dq = (float)RecvLowROS.motorState[i].dq;
        state.motorState[i].ddq = (float)RecvLowROS.motorState[i].ddq;
        state.motorState[i].tauEst = (float)RecvLowROS.motorState[i].tauEst;
        state.motorState[i].q_raw = (float)RecvLowROS.motorState[i].q_raw;
        state.motorState[i].dq_raw = (float)RecvLowROS.motorState[i].dq_raw;
        state.motorState[i].ddq_raw = (float)RecvLowROS.motorState[i].ddq_raw;
        state.motorState[i].temperature = (int8_t)RecvLowROS.motorState[i].temperature;
        state.motorState[i].reserve[0] = (uint32_t)RecvLowROS.motorState[i].reserve[0];
        state.motorState[i].reserve[1] = (uint32_t)RecvLowROS.motorState[i].reserve[1];
    }
    for(int i=0;i<4;i++){
        state.footForce[i] = (int16_t)RecvLowROS.footForce[i];
        state.footForceEst[i] = (int16_t)RecvLowROS.footForceEst[i];
    }
    state.tick = (uint32_t)RecvLowROS.tick;
    for(int i=0;i<40;i++){
        state.wirelessRemote[i] = (uint8_t)RecvLowROS.wirelessRemote[i];
    }
    state.reserve = (uint32_t)RecvLowROS.reserve;
    state.crc = (uint32_t)RecvLowROS.crc;

    cmd.levelFlag = (uint8_t)SendLowROS.levelFlag;
    cmd.commVersion = (uint16_t)SendLowROS.commVersion;
    cmd.robotID = (uint16_t)SendLowROS.robotID;
    cmd.SN = (uint32_t)SendLowROS.SN;
    cmd.bandWidth = (uint8_t)SendLowROS.bandWidth;
    for(int i=0;i<20;i++){
        cmd.motorCmd[i].mode = (uint8_t)SendLowROS.motorCmd[i].mode;
        cmd.motorCmd[i].q = (float)SendLowROS.motorCmd[i].q;
        cmd.motorCmd[i].dq = (float)SendLowROS.motorCmd[i].dq;
        cmd.motorCmd[i].tau = (float)SendLowROS.motorCmd[i].tau;
        cmd.motorCmd[i].Kp = (float)SendLowROS.motorCmd[i].Kp;
        cmd.motorCmd[i].Kd = (float)SendLowROS.motorCmd[i].Kd;
        cmd.motorCmd[i].reserve[0] = (uint32_t)SendLowROS.motorCmd[i].reserve[0];
        cmd.motorCmd[i].reserve[1] = (uint32_t)SendLowROS.motorCmd[i].reserve[1];
        cmd.motorCmd[i].reserve[2] = (uint32_t)SendLowROS.motorCmd[i].reserve[2];
    }
    for(int i=0;i<40;i++){
        cmd.wirelessRemote[i] = (uint8_t)SendLowROS.wirelessRemote[i];
    }
    cmd.reserve = (uint32_t)SendLowROS.reserve;
    cmd.crc = (uint32_t)SendLowROS.crc;

    if(cccount > 3000){
        //safe.PositionLimit(cmd);
        //safe.PowerProtect(cmd, state, 1);
        //safe.PositionProtect(cmd, state, 0.087);
    }
}




template<typename TLCM>
void* update_loop(void* param){
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}
void cmd_callback(const geometry_msgs::Twist& msg_cmd){
    if(flag == 0 && msg_cmd.angular.x == 1){
        c_flag = 1;
        cccount = 0;
        flag = msg_cmd.angular.x;
    }
    else if(flag == 1 && msg_cmd.angular.x == 0){
        c_flag = 2;
        ROS_WARN("stop");
    }
    lin_x = msg_cmd.linear.x;
    lin_y = msg_cmd.linear.y;
    Hz[5] = msg_cmd.linear.z;
    hoge = msg_cmd.angular.y;
    ang_z = msg_cmd.angular.z;
    //std::cout << lin_x <<"/"<< lin_y<<"/"<< lin_z << "/"<< ang_x <<"/"<< ang_y <<"/"<< ang_z << std::endl;
    ROS_INFO("%f,%f,%f,%f,%f,%f",lin_x,lin_y,Hz[5],flag,hoge,ang_z);
}
template<typename TCmd, typename TState, typename TLCM>


int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    double L[5] = {0.0838, 0.2, 0.2, 0, 0};
    double TL[3] = {0.1805, 0.047, 0};
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
    double FR_q[3] = {0};
    double FL_q[3] = {0};
    double RR_q[3] = {0};
    double RL_q[3] = {0};
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    double yaw_save = 0;
    float Kp[3] = {0.0, 0.0, 0.0};  
    float Kd[3] = {0.8, 0.8, 0.8};
    double x_int = 0;
    double y_int = 0;
    double z_int = 0.14;
    double x_ofs = -0.0;
    double y_ofs = 0.0838;
    double z_ofs = -0.168;
    double xR_wave = 0;
    double xL_wave = 0;
    double yR_wave = 0;
    double yL_wave = 0;
    double zR_wave = 0;
    double zL_wave = 0;
    double ang_comp_x = 0;
    double ang_comp_y = 0;
    double r_flag = 0;
    double p1 = 0;
    double p2 = 0;
    double p3 = 0;
    double p4 = 0;
    double p5 = 0;
    double p6 = 0;
    Custom custom(LOWLEVEL);
    double yaw_ofs = 0;
    
    
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    ros::NodeHandle nh;
    ros::Publisher servo_pub[12];
    servo_pub[0] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_calf_controller/command", 1);
    ros::Rate loop_rate(500);
    TCmd SendLowLCM = {0};
    TState RecvLowLCM = {0};
    bool initiated_flag = false;  // initiate need time
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
    ros::Subscriber cmd_sub = nh.subscribe("cmd_vel", 10, cmd_callback);

    while (ros::ok()){
        roslcm.Get(RecvLowLCM);
        RecvLowROS = ToRos(RecvLowLCM);
        ROS_INFO("----------------------");
        std::cout << std::fixed;
        tf::Quaternion quat(RecvLowROS.imu.quaternion[1],RecvLowROS.imu.quaternion[2],RecvLowROS.imu.quaternion[3],RecvLowROS.imu.quaternion[0]);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        //std::cout << "FR_0:" << std::setprecision(5) << RecvLowROS.motorState[FR_0].q * 180 / M_PI << ", FR_1:" << std::setprecision(5) << RecvLowROS.motorState[FR_1].q * 180 / M_PI << ", FR_2:" << std::setprecision(5) << RecvLowROS.motorState[FR_2].q * 180 / M_PI << std::endl;
        //std::cout << "FL_0:" << std::setprecision(5) << RecvLowROS.motorState[FL_0].q * 180 / M_PI << ", FL_1:" << std::setprecision(5) << RecvLowROS.motorState[FL_1].q * 180 / M_PI << ", FL_2:" << std::setprecision(5) << RecvLowROS.motorState[FL_2].q * 180 / M_PI << std::endl;
        //std::cout << "RR_0:" << std::setprecision(5) << RecvLowROS.motorState[RR_0].q * 180 / M_PI << ", RR_1:" << std::setprecision(5) << RecvLowROS.motorState[RR_1].q * 180 / M_PI << ", RR_2:" << std::setprecision(5) << RecvLowROS.motorState[RR_2].q * 180 / M_PI << std::endl;
        //std::cout << "RL_0:" << std::setprecision(5) << RecvLowROS.motorState[RL_0].q * 180 / M_PI << ", RL_1:" << std::setprecision(5) << RecvLowROS.motorState[RL_1].q * 180 / M_PI << ", RL_2:" << std::setprecision(5) << RecvLowROS.motorState[RL_2].q * 180 / M_PI << std::endl;
        //std::cout << "orientation" << ", w:" << std::setprecision(5) << RecvLowROS.imu.quaternion[0] << ", x:" << std::setprecision(5) << RecvLowROS.imu.quaternion[1] << ", y:" << std::setprecision(5) << RecvLowROS.imu.quaternion[2] << ", z:" << std::setprecision(5) << RecvLowROS.imu.quaternion[3]  << std::endl;
        //std::cout << "angular_velocity" << ", x:" << std::setprecision(5) << RecvLowROS.imu.gyroscope[0] << ", y:" << std::setprecision(5) << RecvLowROS.imu.gyroscope[1] << ", z:" << std::setprecision(5) << RecvLowROS.imu.gyroscope[2] << std::endl;
        //std::cout << "linear_acceleration" << ", x:" << std::setprecision(5) << RecvLowROS.imu.accelerometer[0] << ", y:" << std::setprecision(5) << RecvLowROS.imu.accelerometer[1] << ", z:" << std::setprecision(5) << RecvLowROS.imu.accelerometer[2] << std::endl;
        //std::cout << "roll : " << std::setprecision(5) << roll * 180 / M_PI << "/ pitch : " << std::setprecision(5) << pitch * 180 / M_PI << "/ yaw : " << std::setprecision(5) << yaw * 180 / M_PI << std::endl;
        //std::cout << -1 * L[1] * sin(RecvLowROS.motorState[FR_1].q)) + (L[2] * cos(RecvLowROS.motorState[FR_1].q + (M_PI + RecvLowROS.motorState[FR_2].q) - (M_PI/2)) << ":" << -1 * L[1] * sin(RecvLowROS.motorState[RL_1].q)) + (L[2] * cos(RecvLowROS.motorState[RL_1].q + (M_PI + RecvLowROS.motorState[RL_2].q) - (M_PI/2)) << std::endl;

        x_int = lin_x;
        y_int = lin_y;
        Hz[0] = 500.0 / Hz[5];
        Hz[1] = Hz[0] * 0.04;
        Hz[2] = Hz[0] * 0.46;
        Hz[3] = Hz[0] * 0.10;
        Hz[4] = Hz[0] * 0.40;

        if(r_flag <= 5){
            x_int = 0.03 + ((x_int - 0.03) * r_flag / 5);
            y_int = y_int * r_flag / 5;
            ang_z = ang_z * r_flag / 5;
        }


        if(flag == 0 && cccount < Hz[6]){
            Kp[0] = 17 * (-cos(M_PI * cccount/Hz[6]) + 1);
            Kp[1] = 15 * (-cos(M_PI * cccount/Hz[6]) + 1);
            Kp[2] = 15 * (-cos(M_PI * cccount/Hz[6]) + 1);
            zR_wave = cos(M_PI * cccount/Hz[6]);
            zL_wave = cos(M_PI * cccount/Hz[6]);
            std::cout << "stand";
        }
        else if(flag == 0){
            yaw_save = yaw;
            Kp[0] = 34;
            Kp[1] = 30;
            Kp[2] = 30;
            zR_wave = -1;
            zL_wave = -1;
            ROS_WARN("Standby");
            r_flag = 0;
        }
        
        if(flag == 1 && c_flag == 1 && cccount <= Hz[2]/ 2){
            zR_wave = -1;
            zL_wave = -0.5 + (-0.5 * cos(2 * M_PI * (cccount / Hz[2])));
            std::cout << "pL";
            if(cccount == floor(Hz[2]/ 2)){
                c_flag = 0;
                std::cout << "sL";
            }
        }
        else if(flag == 1 && cccount < Hz[2]){
            zR_wave = -1;
            zL_wave = -0.5 + (-0.5 * cos(2 * M_PI * (cccount / Hz[2])));
            std::cout << "L";
        }
        else if(flag == 1 && Hz[1] + Hz[2] < cccount && cccount < Hz[1] + 2*Hz[2]){
            zR_wave = -0.5 + (-0.5 * cos(2 * M_PI * ((cccount - (Hz[1] + Hz[2])) / Hz[2])));
            zL_wave = -1;
            std::cout << "R";
        }
        else if(flag == 1){
            zR_wave = -1;
            zL_wave = -1;
            std::cout << "s";
        }

        if(c_flag == 3 && cccount == 0){
            cccount = Hz[6];
            zR_wave = -1;
            zL_wave = -1;
            flag = 0;
            c_flag = 0;
            ROS_ERROR("ZStandby");
        }

        if(c_flag == 2 && (cccount == floor(Hz[4] / 2.0) || cccount == floor((Hz[0] + Hz[4]) / 2.0)) ){
            xR_wave = 0;
            xL_wave = 0;
            yR_wave = 0;
            yL_wave = 0;
            ROS_ERROR("XYStandby");
            c_flag = 3;
        }
        else if(c_flag == 3){
            xR_wave = 0;
            xL_wave = 0;
            yR_wave = 0;
            yL_wave = 0;
            ROS_WARN("XYStandby");
        }
        else if(flag == 1 && c_flag != 1 && cccount <= Hz[4]){
            xR_wave = cos(M_PI * (cccount/Hz[4]));
            xL_wave = cos(M_PI * (cccount/Hz[4]));
            yR_wave = cos(M_PI * (cccount/Hz[4]));
            yL_wave = cos(M_PI * (cccount/Hz[4]));
            std::cout << "L";
        }
        else if(flag == 1 && c_flag != 1 && Hz[4] < cccount && cccount < Hz[0]/2){
            xR_wave = -1;
            xL_wave = -1;
            yR_wave = -1;
            yL_wave = -1;
            std::cout << "l";
        }
        else if(flag == 1 && c_flag != 1 && Hz[0]/2 <= cccount && cccount <= Hz[0]/2 + Hz[4]){
            xR_wave = -1 * cos(M_PI * ((cccount-(Hz[0]/2))/Hz[4]));
            xL_wave = -1 * cos(M_PI * ((cccount-(Hz[0]/2))/Hz[4]));
            yR_wave = -1 * cos(M_PI * ((cccount-(Hz[0]/2))/Hz[4]));
            yL_wave = -1 * cos(M_PI * ((cccount-(Hz[0]/2))/Hz[4]));
            std::cout << "R";
        }
        else if(flag == 1 && c_flag != 1 && Hz[0]/2 + Hz[4] < cccount){
            xR_wave = 1;
            xL_wave = 1;
            yR_wave = 1;
            yL_wave = 1;
            std::cout << "r";
        }
        else if(flag == 1){
            xR_wave = 0;
            xL_wave = 0;
            yR_wave = 0;
            yL_wave = 0;
            std::cout << "s";
        }

        //初期姿勢から見た正しいyawの算出
        std::cout << cccount << std::endl;
        //std::cout << "yaw : " << std::endl;
        

        yaw = (yaw_save - yaw);
        if(yaw > M_PI){yaw = yaw - (2 * M_PI);}
        else if(yaw < (-1 * M_PI)){yaw = yaw + (2 * M_PI);}
        //std::cout << "yaw : " << std::setprecision(5) << yaw * 180 / M_PI << ", r_flag : " << r_flag << ", c_flag : " << c_flag <<  std::endl;
        
        //rpyの応答性と上限値の設定
        roll = roll / 3;
        pitch = pitch / 3;
        yaw = (yaw / 2) + ang_z;
        roll = 0;
        pitch = 0;
        yaw = 0 ;

        if(roll < -1 * M_PI / 20){roll = -1 * M_PI / 20;}
        else if(roll > M_PI / 20){roll = M_PI / 20;}
        if(pitch < -1 * M_PI / 20){pitch = -1 * M_PI / 20;}
        else if(pitch > M_PI / 20){pitch = M_PI / 20;}
        if((yaw * 180 / M_PI) > 30){yaw = 30 * M_PI / 180;}
        else if((yaw * 180 / M_PI) < -30){yaw = -1 * 30 * M_PI / 180;}
        
        //yaw方向の回転運動生成
        ang_comp_x = (((x_ofs + TL[0]) * cos(yaw)) + ((y_ofs + TL[1]) * sin(yaw)) - (x_ofs + TL[0])) / 2;
        ang_comp_y = (((y_ofs + TL[1]) * cos(yaw)) - ((x_ofs + TL[0]) * sin(yaw)) - (y_ofs + TL[1])) / 2;

        //各足の位置計算
        FRb_x = x_ofs + ((x_int + ang_comp_x) * xR_wave);
        FLb_x = x_ofs - ((x_int - ang_comp_x) * xL_wave);
        RRb_x = x_ofs - ((x_int + ang_comp_x) * xL_wave);
        RLb_x = x_ofs + ((x_int - ang_comp_x) * xR_wave);

        FRb_y = y_ofs + ((y_int + ang_comp_y) * yR_wave);
        FLb_y = -1 * y_ofs - ((y_int + ang_comp_y) * yL_wave);
        RRb_y = y_ofs - ((y_int - ang_comp_y) * yL_wave);
        RLb_y = -1 * y_ofs + ((y_int - ang_comp_y) * yR_wave);
        
        FRb_z = z_ofs + (z_int * zR_wave);
        FLb_z = z_ofs + (z_int * zL_wave);
        RRb_z = z_ofs + (z_int * zL_wave);
        RLb_z = z_ofs + (z_int * zR_wave);
        
        //std::cout << FR_x << ":" << FL_x << std::endl;
        //std::cout << FR_y << ":" << FL_y << std::endl;
        //std::cout << FR_z << ":" << FL_z << std::endl;

        FR_x =  (FRb_x * cos(pitch)) - (FRb_z * sin(pitch));
        FL_x =  (FLb_x * cos(pitch)) - (FLb_z * sin(pitch));
        RR_x =  (RRb_x * cos(pitch)) - (RRb_z * sin(pitch));
        RL_x =  (RLb_x * cos(pitch)) - (RLb_z * sin(pitch));
        FR_y =  (FRb_y * cos(roll)) - (FRb_z * sin(roll));
        FL_y =  (FLb_y * cos(roll)) - (FLb_z * sin(roll));
        RR_y =  (RRb_y * cos(roll)) - (RRb_z * sin(roll));
        RL_y =  (RLb_y * cos(roll)) - (RLb_z * sin(roll));
        FR_z =  (abs(FRb_x * sin(pitch)) + (FRb_z * cos(pitch)) + abs(FRb_y * sin(roll)) + (FRb_z * cos(roll))) / 2;
        FL_z =  (abs(FLb_x * sin(pitch)) + (FLb_z * cos(pitch)) + abs(FLb_y * sin(roll)) + (FLb_z * cos(roll))) / 2;
        RR_z =  (abs(RRb_x * sin(pitch)) + (RRb_z * cos(pitch)) + abs(RRb_y * sin(roll)) + (RRb_z * cos(roll))) / 2;
        RL_z =  (abs(RLb_x * sin(pitch)) + (RLb_z * cos(pitch)) + abs(RLb_y * sin(roll)) + (RLb_z * cos(roll))) / 2;

        //std::cout << lin_x <<"/"<< lin_y<<"/"<< Hz[5] << "/"<< flag <<"/"<< hoge <<"/"<< ang_z << std::endl;

        //double FR1 = FR_q[1];
        //double FR2 = FR_q[2];
        

    {
        L[3] = std::pow(FR_x, 2.0)+std::pow(FR_y, 2.0)+std::pow(FR_z, 2.0);
        FR_q[2] = -acos((L[3]-std::pow(L[0], 2.0)-std::pow(L[1], 2.0)-std::pow(L[2], 2.0)) / (2*L[1]*L[2]));
        FR_q[1] = -asin(FR_x / std::pow(L[3]-std::pow(L[0], 2.0), 0.5)) - atan((L[2]*sin(FR_q[2])) / (L[1]+(L[2]*cos(FR_q[2]))));
        L[4] = (L[1]*cos(FR_q[1])) + (L[2]*cos(FR_q[1])*cos(FR_q[2])) - (L[2]*sin(FR_q[1])*sin(FR_q[2]));
        FR_q[0] = (-1 * asin(FR_y / std::pow(std::pow(L[0], 2.0)+std::pow(L[4], 2.0), 0.5))) + (atan(L[0]/L[4]));
        
        L[3] = std::pow(FL_x, 2.0)+std::pow(FL_y, 2.0)+std::pow(FL_z, 2.0);
        FL_q[2] = -acos((L[3]-std::pow(L[0], 2.0)-std::pow(L[1], 2.0)-std::pow(L[2], 2.0)) / (2*L[1]*L[2]));
        FL_q[1] = -asin(FL_x / std::pow(L[3]-std::pow(L[0], 2.0), 0.5)) - atan((L[2]*sin(FL_q[2])) / (L[1]+(L[2]*cos(FL_q[2]))));
        L[4] = (L[1]*cos(FL_q[1])) + (L[2]*cos(FL_q[1])*cos(FL_q[2])) - (L[2]*sin(FL_q[1])*sin(FL_q[2]));
        FL_q[0] = (-1 * asin(FL_y / std::pow(std::pow(L[0], 2.0)+std::pow(L[4], 2.0), 0.5))) - (atan(L[0]/L[4]));

        L[3] = std::pow(RR_x, 2.0)+std::pow(RR_y, 2.0)+std::pow(RR_z, 2.0);
        RR_q[2] = -acos((L[3]-std::pow(L[0], 2.0)-std::pow(L[1], 2.0)-std::pow(L[2], 2.0)) / (2*L[1]*L[2]));
        RR_q[1] = -asin(RR_x / std::pow(L[3]-std::pow(L[0], 2.0), 0.5)) - atan((L[2]*sin(RR_q[2])) / (L[1]+(L[2]*cos(RR_q[2]))));
        L[4] = (L[1]*cos(RR_q[1])) + (L[2]*cos(RR_q[1])*cos(RR_q[2])) - (L[2]*sin(RR_q[1])*sin(RR_q[2]));
        RR_q[0] = (-1 * asin(RR_y / std::pow(std::pow(L[0], 2.0)+std::pow(L[4], 2.0), 0.5))) + (atan(L[0]/L[4]));

        L[3] = std::pow(RL_x, 2.0)+std::pow(RL_y, 2.0)+std::pow(RL_z, 2.0);
        RL_q[2] = -acos((L[3]-std::pow(L[0], 2.0)-std::pow(L[1], 2.0)-std::pow(L[2], 2.0)) / (2*L[1]*L[2]));
        RL_q[1] = -asin(RL_x / std::pow(L[3]-std::pow(L[0], 2.0), 0.5)) - atan((L[2]*sin(RL_q[2])) / (L[1]+(L[2]*cos(RL_q[2]))));
        L[4] = (L[1]*cos(RL_q[1])) + (L[2]*cos(RL_q[1])*cos(RL_q[2])) - (L[2]*sin(RL_q[1])*sin(RL_q[2]));
        RL_q[0] = (-1 * asin(RL_y / std::pow(std::pow(L[0], 2.0)+std::pow(L[4], 2.0), 0.5))) - (atan(L[0]/L[4]));
        
        std::cout << FR_q[0] * 180 / M_PI << ";" << FR_q[1] * 180 / M_PI << ";" << FR_q[2] * 180 / M_PI << std::endl;
        //std::cout << FL_q[0] * 180 / M_PI << ";" << FL_q[1] * 180 / M_PI << ";" << FL_q[2] * 180 / M_PI << std::endl;
        //std::cout << RR_q[0] * 180 / M_PI << ";" << RR_q[1] * 180 / M_PI << ";" << RR_q[2] * 180 / M_PI << std::endl;
        //std::cout << RL_q[0] * 180 / M_PI << ";" << RL_q[1] * 180 / M_PI << ";" << RL_q[2] * 180 / M_PI << std::endl;

        if(FR_q[0] > 0.8){FR_q[0] = 0.8;}//ROS_WARN("1Angle command value is out of range");}
        if(FR_q[0] < -0.8){FR_q[0] = -0.8;}//ROS_WARN("2Angle command value is out of range");}
        if(FL_q[0] > 0.8){FL_q[0] = 0.8;}//ROS_WARN("3Angle command value is out of range");}
        if(FL_q[0] < -0.8){FL_q[0] = -0.8;}//ROS_WARN("4Angle command value is out of range");}
        if(RR_q[0] > 0.8){RR_q[0] = 0.8;}//ROS_WARN("5Angle command value is out of range");}
        if(RR_q[0] < -0.8){RR_q[0] = -0.8;}//ROS_WARN("6Angle command value is out of range");}
        if(RL_q[0] > 0.8){RL_q[0] = 0.8;}//ROS_WARN("7Angle command value is out of range");}
        if(RL_q[0] < -0.8){RL_q[0] = -0.8;}//ROS_WARN("8Angle command value is out of range");}

        if(FR_q[1] > 4.0){FR_q[1] = 4.0;}//ROS_WARN("9Angle command value is out of range");}
        if(FR_q[1] < -1.0){FR_q[1] = -1.0;}//ROS_WARN("10Angle command value is out of range");}
        if(FL_q[1] > 4.0){FL_q[1] = 4.0;}//ROS_WARN("11Angle command value is out of range");}
        if(FL_q[1] < -1.0){FL_q[1] = -1.0;}//ROS_WARN("12Angle command value is out of range");}
        if(RR_q[1] > 4.0){RR_q[1] = 4.0;}//ROS_WARN("13Angle command value is out of range");}
        if(RR_q[1] < -1.0){RR_q[1] = -1.0;}//ROS_WARN("14Angle command value is out of range");}
        if(RL_q[1] > 4.0){RL_q[1] = 4.0;}//ROS_WARN("15Angle command value is out of range");}
        if(RL_q[1] < -1.0){RL_q[1] = -1.0;}//ROS_WARN("16Angle command value is out of range");}
        
        if(FR_q[2] > -1.0){FR_q[2] = -1.0;}//ROS_WARN("17Angle command value is out of range");}
        if(FR_q[2] < -2.6){FR_q[2] = -2.6;}//ROS_WARN("18Angle command value is out of range");}
        if(FL_q[2] > -1.0){FL_q[2] = -1.0;}//ROS_WARN("19Angle command value is out of range");}
        if(FL_q[2] < -2.6){FL_q[2] = -2.6;}//ROS_WARN("20Angle command value is out of range");}
        if(RR_q[2] > -1.0){RR_q[2] = -1.0;}//ROS_WARN("21Angle command value is out of range");}
        if(RR_q[2] < -2.6){RR_q[2] = -2.6;}//ROS_WARN("22Angle command value is out of range");}
        if(RL_q[2] > -1.0){RL_q[2] = -1.0;}//ROS_WARN("23Angle command value is out of range");}
        if(RL_q[2] < -2.6){RL_q[2] = -2.6;}//ROS_WARN("24Angle command value is out of range");}

        //std::cout << abs(FR_q[1]) << std::endl;
        //std::cout << abs(FR1) << std::endl;
        //std::cout << abs(FR_q[2]) << std::endl;
        //std::cout << abs(FR2) << std::endl;
        //std::cout << abs(FR_q[1] - FR1) << std::endl;
        //std::cout << abs(FR_q[2] - FR2) << std::endl;
        
        /*if(0.04 < abs(FR_q[1] - FR1) || 0.04 < abs(FR_q[2] - FR2)){
            ROS_ERROR("auti");
        }*/
        
        //FR
        {
        SendLowROS.motorCmd[FR_0].q = (float)FR_q[0];
        SendLowROS.motorCmd[FR_0].dq = 0;
        SendLowROS.motorCmd[FR_0].Kp = Kp[0];
        SendLowROS.motorCmd[FR_0].Kd = Kd[0];
        SendLowROS.motorCmd[FR_0].tau = -1.0f;

        SendLowROS.motorCmd[FR_1].q = (float)FR_q[1];
        SendLowROS.motorCmd[FR_1].dq = 0;
        SendLowROS.motorCmd[FR_1].Kp = Kp[1];
        SendLowROS.motorCmd[FR_1].Kd = Kd[1];
        SendLowROS.motorCmd[FR_1].tau = 0.0f;

        SendLowROS.motorCmd[FR_2].q =  (float)FR_q[2];
        SendLowROS.motorCmd[FR_2].dq = 0;
        SendLowROS.motorCmd[FR_2].Kp = Kp[2];
        SendLowROS.motorCmd[FR_2].Kd = Kd[2];
        SendLowROS.motorCmd[FR_2].tau = 0.0f;}
        //FL
        {
        SendLowROS.motorCmd[FL_0].q = (float)FL_q[0];
        SendLowROS.motorCmd[FL_0].dq = 0;
        SendLowROS.motorCmd[FL_0].Kp = Kp[0];
        SendLowROS.motorCmd[FL_0].Kd = Kd[0];
        SendLowROS.motorCmd[FL_0].tau = 1.0f;

        SendLowROS.motorCmd[FL_1].q = (float)FL_q[1];
        SendLowROS.motorCmd[FL_1].dq = 0;
        SendLowROS.motorCmd[FL_1].Kp = Kp[1];
        SendLowROS.motorCmd[FL_1].Kd = Kd[1];
        SendLowROS.motorCmd[FL_1].tau = 0.0f;

        SendLowROS.motorCmd[FL_2].q =  (float)FL_q[2];
        SendLowROS.motorCmd[FL_2].dq = 0;
        SendLowROS.motorCmd[FL_2].Kp = Kp[2];
        SendLowROS.motorCmd[FL_2].Kd = Kd[2];
        SendLowROS.motorCmd[FL_2].tau = 0.0f;}
        //RR
        {
        SendLowROS.motorCmd[RR_0].q = (float)RR_q[0];
        SendLowROS.motorCmd[RR_0].dq = 0;
        SendLowROS.motorCmd[RR_0].Kp = Kp[0];
        SendLowROS.motorCmd[RR_0].Kd = Kd[0];
        SendLowROS.motorCmd[RR_0].tau = -1.0f;

        SendLowROS.motorCmd[RR_1].q = (float)RR_q[1];
        SendLowROS.motorCmd[RR_1].dq = 0;
        SendLowROS.motorCmd[RR_1].Kp = Kp[1];
        SendLowROS.motorCmd[RR_1].Kd = Kd[1];
        SendLowROS.motorCmd[RR_1].tau = 0.0f;

        SendLowROS.motorCmd[RR_2].q =  (float)RR_q[2];
        SendLowROS.motorCmd[RR_2].dq = 0;
        SendLowROS.motorCmd[RR_2].Kp = Kp[2];
        SendLowROS.motorCmd[RR_2].Kd = Kd[2];
        SendLowROS.motorCmd[RR_2].tau = 0.0f;}
        //RL
        {
        SendLowROS.motorCmd[RL_0].q = (float)RL_q[0];
        SendLowROS.motorCmd[RL_0].dq = 0;
        SendLowROS.motorCmd[RL_0].Kp = Kp[0];
        SendLowROS.motorCmd[RL_0].Kd = Kd[0];
        SendLowROS.motorCmd[RL_0].tau = 1.0f;

        SendLowROS.motorCmd[RL_1].q = (float)RL_q[1];
        SendLowROS.motorCmd[RL_1].dq = 0;
        SendLowROS.motorCmd[RL_1].Kp = Kp[1];
        SendLowROS.motorCmd[RL_1].Kd = Kd[1];
        SendLowROS.motorCmd[RL_1].tau = 0.0f;

        SendLowROS.motorCmd[RL_2].q =  (float)RL_q[2];
        SendLowROS.motorCmd[RL_2].dq = 0;
        SendLowROS.motorCmd[RL_2].Kp = Kp[2];
        SendLowROS.motorCmd[RL_2].Kd = Kd[2];
        SendLowROS.motorCmd[RL_2].tau = 0.0f;}

        custom.RobotSafe();

        //gazebo
        {
        servo_pub[0].publish(SendLowROS.motorCmd[FR_0]);
        servo_pub[1].publish(SendLowROS.motorCmd[FR_1]);
        servo_pub[2].publish(SendLowROS.motorCmd[FR_2]);
        servo_pub[3].publish(SendLowROS.motorCmd[FL_0]);
        servo_pub[4].publish(SendLowROS.motorCmd[FL_1]);
        servo_pub[5].publish(SendLowROS.motorCmd[FL_2]);
        servo_pub[6].publish(SendLowROS.motorCmd[RR_0]);
        servo_pub[7].publish(SendLowROS.motorCmd[RR_1]);
        servo_pub[8].publish(SendLowROS.motorCmd[RR_2]);
        servo_pub[9].publish(SendLowROS.motorCmd[RL_0]);
        servo_pub[10].publish(SendLowROS.motorCmd[RL_1]);
        servo_pub[11].publish(SendLowROS.motorCmd[RL_2]);}


        //std::cout << RecvLowROS.motorState[FR_1].q * 180 / M_PI << "," << SendLowROS.motorCmd[FR_1].q * 180 / M_PI << "," << RecvLowROS.motorState[FR_2].q * 180 / M_PI << "," << SendLowROS.motorCmd[FR_2].q * 180 / M_PI << std::endl;
    
    }
        
        cccount++;
        if(flag != 0 && Hz[0] <= cccount){
            cccount = 0;
            r_flag = r_flag + 1;
            std::cout << r_flag << std::endl; 
        }
        SendLowLCM = ToLcm(SendLowROS, SendLowLCM);
        roslcm.Send(SendLowLCM);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}



int main(int argc, char *argv[]){
    ros::init(argc, argv, "position_ros_mode");
    std::string firmwork;
    ros::param::get("/firmwork", firmwork);
    std::string robot_name;
    UNITREE_LEGGED_SDK::LeggedType rname;
    ros::param::get("/robot_name", robot_name);
    rname = UNITREE_LEGGED_SDK::LeggedType::A1;
    UNITREE_LEGGED_SDK::LCM roslcm(LOWLEVEL);
    mainHelper<UNITREE_LEGGED_SDK::LowCmd, UNITREE_LEGGED_SDK::LowState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
}