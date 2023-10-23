#include <ros/ros.h>
#include <string>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "convert.h"
#include <tf/transform_broadcaster.h>
//ちょっとあがってからサイン波
#ifdef SDK3_1
using namespace aliengo;
#endif
#ifdef SDK3_2
using namespace UNITREE_LEGGED_SDK;
#endif

template<typename TLCM>
void* update_loop(void* param){
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
    double Hz[4] = {260, 0 ,0 , 2500};//{4の倍数, 0, 0, 立ち上がり時間}
    Hz[1] = Hz[0] * 0.4;
    Hz[2] = Hz[0] * 0.1;
    long count = 0;
    double flag = 0;
    double FR_q[3] = {0};
    double FL_q[3] = {0};
    double RR_q[3] = {0};
    double RL_q[3] = {0};
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    double yaw_save = 0;
    double Kp_fin = 30;
    float Kp[3] = {0.0, 0.0, 0.0};  
    float Kd[3] = {0.8, 0.8, 0.8};
    double p1 = 0;
    double p2 = 0;
    double p3 = 0;
    double p4 = 0;
    double p5 = 0;
    double p6 = 0;
    double x_int = 0.1;
    double y_int = 0;
    double z_int = 0.15;
    double x_ofs = -0.05;
    double y_ofs = 0.0838;
    double z_ofs = -0.15;
    double x_wave = 0;
    double xR_wave = 0;
    double xL_wave = 0;
    double y_wave = 0;
    double zR_wave = 0;
    double zL_wave = 0;
    double yaw_max = 0;
    double yaw_min = 0;
    
    
    TCmd SendLowLCM = {0};
    TState RecvLowLCM = {0};
    unitree_legged_msgs::LowCmd SendLowROS;
    unitree_legged_msgs::LowState RecvLowROS;
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

    while (ros::ok()){
        roslcm.Get(RecvLowLCM);
        RecvLowROS = ToRos(RecvLowLCM);
        count++;

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
        //std::cout << count << std::endl;

        roll = roll / 1.5;
        pitch = pitch / 1.5;
        yaw = yaw / 2;
        
        if(roll < -1 * M_PI / 20){roll = -1 * M_PI / 20;}
        else if(roll > M_PI / 20){roll = M_PI / 20;}
        if(pitch < -1 * M_PI / 20){pitch = -1 * M_PI / 20;}
        else if(pitch > M_PI / 20){pitch = M_PI / 20;}

        if(flag == 0 && count < Hz[3]){
            Kp[0] = 15 * (-cos(M_PI * count/Hz[3]) + 1);
            Kp[1] = 15 * (-cos(M_PI * count/Hz[3]) + 1);
            Kp[2] = 15 * (-cos(M_PI * count/Hz[3]) + 1);
            zR_wave = cos(M_PI * count/Hz[3]);
            zL_wave = cos(M_PI * count/Hz[3]);
            yaw_save = yaw;
        }
        else if(flag == 0 && Hz[3] <= count && count < (Hz[1] / 2) + Hz[3]){
            Kp[0] = 30;
            Kp[1] = 30;
            Kp[2] = 30;
            zR_wave = -1;
            zL_wave = -0.5 + (-0.5 * cos(2 * M_PI * ((count - Hz[3])/Hz[1])));
        }
        else if(flag == 0 && (Hz[1] / 2) + Hz[3] <= count){
            ROS_ERROR("reset");
            flag = 1;
            count = count - Hz[3];
            yaw_max = yaw_save;
            yaw_min = yaw_save;
        }

        if(flag == 1 && 0 <= count && count < Hz[1]){
            xR_wave = cos(M_PI * (count/Hz[1]));
            if(Hz[1] * 0.1 <= count && count <= Hz[1] * 0.9){
                xL_wave = cos(M_PI * ((count -(Hz[1] * 0.1))/(Hz[1] * 0.8)));    
            }
            y_wave = cos(M_PI * (count/Hz[1]));
            zR_wave = -1;
            zL_wave = -0.5 + (-0.5 * cos(2 * M_PI * (count/Hz[1])));
            std::cout << "A";
        }
        else if(flag == 1 && Hz[1] <= count && count < Hz[1] + Hz[2]){
            xR_wave = -1;
            xL_wave = -1;
            y_wave = -1;
            zR_wave = -1;
            zL_wave = -1;    
            std::cout << "B";
        }
        else if(flag == 1 && Hz[1] + Hz[2] <= count && count < (2 * Hz[1]) + Hz[2]){
            if(Hz[1] + Hz[2] + (Hz[1] * 0.1) <= count && count <= Hz[1] + Hz[2] + (Hz[1] * 0.9)){
                xR_wave = -1 * cos(M_PI * ((count -(Hz[1] + Hz[2] + (Hz[1] * 0.1)))/(Hz[1] * 0.8)));    
            }
            xL_wave = -1 * cos(M_PI * ((count - Hz[1] - Hz[2])/Hz[1]));
            y_wave = -1 * cos(M_PI * ((count - Hz[1] - Hz[2])/Hz[1]));
            zR_wave = -0.5 + (-0.5 * cos(2 * M_PI * ((count - Hz[1] - Hz[2]) / Hz[1])));
            zL_wave = -1;  
            std::cout << "C";
        }
        else if(flag == 1 && (2 * Hz[1]) + Hz[2] <= count && count < 2 * (Hz[1] + Hz[2])){
            xR_wave = 1;
            xL_wave = 1;
            y_wave = 1;
            zR_wave = -1;
            zL_wave = -1;
            std::cout << "D";
        }
        else if(flag == 1 && Hz[0] <= count){
            count = 0;
            ROS_WARN("reset");
            xR_wave = 1;
            xL_wave = 1;
            y_wave = 1;
            zR_wave = -1;
            zL_wave = -1;
            std::cout << "E";
        }

        yaw = (yaw_save - yaw);
        //std::cout << "yaw : " << std::setprecision(5) << yaw * 180 / M_PI << std::endl;
        if(yaw > M_PI){
            yaw = yaw - (2 * M_PI);
        }
        else if(yaw < (-1 * M_PI)){
            yaw = yaw + (2 * M_PI);
        }
        //std::cout << "yaw : " << std::setprecision(5) << yaw * 180 / M_PI << std::endl;
        //yaw = yaw * 4;
        //yaw = 0;
        //yaw = 4 * sin(2 * M_PI * ((count  /Hz[0])) * M_PI / 180;




        if((yaw * 180 / M_PI) > 30){
            yaw = 30 * M_PI / 180;
        }
        else if((yaw * 180 / M_PI) < -30){
            yaw = -1 * 30 * M_PI / 180;
        }
        
        p5 = (((x_ofs + TL[0]) * cos(yaw)) + ((y_ofs + TL[1]) * sin(yaw)) - (x_ofs + TL[0])) / 2;
        p6 = (((y_ofs + TL[1]) * cos(yaw)) - ((x_ofs + TL[0]) * sin(yaw)) - (y_ofs + TL[1])) / 2;
        //x_int = 0.04;
        //p5 = 0;
        //p6 = 0;

        p3 = (-1 * L[1] * sin(RecvLowROS.motorState[FR_1].q)) + (L[2] * cos(RecvLowROS.motorState[FR_1].q + (M_PI + RecvLowROS.motorState[FR_2].q) - (M_PI/2)));;
        p4 = (-1 * L[1] * sin(RecvLowROS.motorState[RL_1].q)) + (L[2] * cos(RecvLowROS.motorState[RL_1].q + (M_PI + RecvLowROS.motorState[RL_2].q) - (M_PI/2)));;
        
        if(flag == 0){
            FRb_x = x_ofs + (x_int * x_wave);
            FLb_x = x_ofs - (x_int * x_wave);
            RRb_x = x_ofs - (x_int * x_wave);
            RLb_x = x_ofs + (x_int * x_wave);
            FRb_y = y_ofs + (y_int * y_wave);
            FLb_y = -1 * y_ofs - (y_int * y_wave);
            RRb_y = y_ofs - (y_int * y_wave);
            RLb_y = -1 * y_ofs + (y_int * y_wave);
        }
        if(flag == 1){
            FRb_x = x_ofs + ((x_int + p5) * xR_wave);
            FLb_x = x_ofs - ((x_int - p5) * xL_wave);
            RRb_x = x_ofs - ((x_int + p5) * xL_wave);
            RLb_x = x_ofs + ((x_int - p5) * xR_wave);
            FRb_y = y_ofs + ((y_int + p6) * y_wave);
            FLb_y = -1 * y_ofs - ((y_int + p6) * y_wave);
            RRb_y = y_ofs - ((y_int - p6) * y_wave);
            RLb_y = -1 * y_ofs + ((y_int - p6) * y_wave);
        }

        FRb_z = z_ofs + (z_int * zR_wave);
        FLb_z = z_ofs + (z_int * zL_wave);
        RRb_z = z_ofs + (z_int * zL_wave);
        RLb_z = z_ofs + (z_int * zR_wave);

        std::cout << count << std::endl;

        //std::cout << count << ":" << Hz[1] << ":" << Hz[2] << std::endl;
        //std::cout << FR_x << std::endl;
        //std::cout << Kp[0] << std::endl;
        //std::cout << FR_z << ":" << p6 << std::endl;
        //std::cout << FR_x << ":" << FL_x << ";" << RR_x << ":" << RL_x << std::endl;
        
        //std::cout << "yaw : " << std::setprecision(5) << yaw * 180 / M_PI << std::endl;
        std::cout << FR_x << ":" << RL_x << std::endl;
        //std::cout << FR_y << ":" << FL_y << std::endl;
        //std::cout << FR_z << ":" << FL_z << std::endl;
        std::cout << p3 << ":" << p4 << std::endl;


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
    }
    


    {    
        //std::cout << FR_q[0] * 180 / M_PI << ";" << FR_q[1] * 180 / M_PI << ";" << FR_q[2] * 180 / M_PI << std::endl;
        //std::cout << FL_q[0] * 180 / M_PI << ";" << FL_q[1] * 180 / M_PI << ";" << FL_q[2] * 180 / M_PI << std::endl;
        //std::cout << RR_q[0] * 180 / M_PI << ";" << RR_q[1] * 180 / M_PI << ";" << RR_q[2] * 180 / M_PI << std::endl;
        //std::cout << RL_q[0] * 180 / M_PI << ";" << RL_q[1] * 180 / M_PI << ";" << RL_q[2] * 180 / M_PI << std::endl;

        if(FR_q[0] > 0.8){FR_q[0] = 0.8;ROS_WARN("1Angle command value is out of range");}
        if(FR_q[0] < -0.8){FR_q[0] = -0.8;ROS_WARN("2Angle command value is out of range");}
        if(FL_q[0] > 0.8){FL_q[0] = 0.8;ROS_WARN("3Angle command value is out of range");}
        if(FL_q[0] < -0.8){FL_q[0] = -0.8;ROS_WARN("4Angle command value is out of range");}
        if(RR_q[0] > 0.8){RR_q[0] = 0.8;ROS_WARN("5Angle command value is out of range");}
        if(RR_q[0] < -0.8){RR_q[0] = -0.8;ROS_WARN("6Angle command value is out of range");}
        if(RL_q[0] > 0.8){RL_q[0] = 0.8;ROS_WARN("7Angle command value is out of range");}
        if(RL_q[0] < -0.8){RL_q[0] = -0.8;ROS_WARN("8Angle command value is out of range");}

        if(FR_q[1] > 4.0){FR_q[1] = 4.0;ROS_WARN("9Angle command value is out of range");}
        if(FR_q[1] < -1.0){FR_q[1] = -1.0;ROS_WARN("10Angle command value is out of range");}
        if(FL_q[1] > 4.0){FL_q[1] = 4.0;ROS_WARN("11Angle command value is out of range");}
        if(FL_q[1] < -1.0){FL_q[1] = -1.0;ROS_WARN("12Angle command value is out of range");}
        if(RR_q[1] > 4.0){RR_q[1] = 4.0;ROS_WARN("13Angle command value is out of range");}
        if(RR_q[1] < -1.0){RR_q[1] = -1.0;ROS_WARN("14Angle command value is out of range");}
        if(RL_q[1] > 4.0){RL_q[1] = 4.0;ROS_WARN("15Angle command value is out of range");}
        if(RL_q[1] < -1.0){RL_q[1] = -1.0;ROS_WARN("16Angle command value is out of range");}
        
        if(FR_q[2] > -1.0){FR_q[2] = -1.0;ROS_WARN("17Angle command value is out of range");}
        if(FR_q[2] < -2.6){FR_q[2] = -2.6;ROS_WARN("18Angle command value is out of range");}
        if(FL_q[2] > -1.0){FL_q[2] = -1.0;ROS_WARN("19Angle command value is out of range");}
        if(FL_q[2] < -2.6){FL_q[2] = -2.6;ROS_WARN("20Angle command value is out of range");}
        if(RR_q[2] > -1.0){RR_q[2] = -1.0;ROS_WARN("21Angle command value is out of range");}
        if(RR_q[2] < -2.6){RR_q[2] = -2.6;ROS_WARN("22Angle command value is out of range");}
        if(RL_q[2] > -1.0){RL_q[2] = -1.0;ROS_WARN("23Angle command value is out of range");}
        if(RL_q[2] < -2.6){RL_q[2] = -2.6;ROS_WARN("24Angle command value is out of range");}
    }

    {
        //FR
        {
        SendLowROS.motorCmd[FR_0].q = (float)FR_q[0];
        SendLowROS.motorCmd[FR_0].dq = 0;
        SendLowROS.motorCmd[FR_0].Kp = Kp[0];
        SendLowROS.motorCmd[FR_0].Kd = Kd[0];
        SendLowROS.motorCmd[FR_0].tau = -1;

        SendLowROS.motorCmd[FR_1].q = (float)FR_q[1];
        SendLowROS.motorCmd[FR_1].dq = 0;
        SendLowROS.motorCmd[FR_1].Kp = Kp[1];
        SendLowROS.motorCmd[FR_1].Kd = Kd[1];
        SendLowROS.motorCmd[FR_1].tau = 0.0;

        SendLowROS.motorCmd[FR_2].q =  (float)FR_q[2];
        SendLowROS.motorCmd[FR_2].dq = 0;
        SendLowROS.motorCmd[FR_2].Kp = Kp[2];
        SendLowROS.motorCmd[FR_2].Kd = Kd[2];
        SendLowROS.motorCmd[FR_2].tau = 0.0;}
        //FL
        {
        SendLowROS.motorCmd[FL_0].q = (float)FL_q[0];
        SendLowROS.motorCmd[FL_0].dq = 0;
        SendLowROS.motorCmd[FL_0].Kp = Kp[0];
        SendLowROS.motorCmd[FL_0].Kd = Kd[0];
        SendLowROS.motorCmd[FL_0].tau = 1;

        SendLowROS.motorCmd[FL_1].q = (float)FL_q[1];
        SendLowROS.motorCmd[FL_1].dq = 0;
        SendLowROS.motorCmd[FL_1].Kp = Kp[1];
        SendLowROS.motorCmd[FL_1].Kd = Kd[1];
        SendLowROS.motorCmd[FL_1].tau = 0.0;

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
        SendLowROS.motorCmd[RR_0].tau = -1;

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
        SendLowROS.motorCmd[RL_0].tau = 1;

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