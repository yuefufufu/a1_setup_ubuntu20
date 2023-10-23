#include <ros/ros.h>
#include <string>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "convert.h"
#include <tf/transform_broadcaster.h>
//両足設置なし
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
    double Hz[6] = {250, 0 ,0 , 2500, 0, 0};//{4の倍数, 0, 0, 立ち上がり時間}
    Hz[1] = Hz[0] * 0.35;
    Hz[2] = Hz[0] * 0.15;
    Hz[4] = Hz[0] * 0.48;
    Hz[5] = Hz[0] * 0.02;
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

        std::cout << "FR_0:" << std::setprecision(5) << RecvLowROS.motorState[FR_0].q * 180 / M_PI << ", FR_1:" << std::setprecision(5) << RecvLowROS.motorState[FR_1].q * 180 / M_PI << ", FR_2:" << std::setprecision(5) << RecvLowROS.motorState[FR_2].q * 180 / M_PI << std::endl;
        std::cout << "FL_0:" << std::setprecision(5) << RecvLowROS.motorState[FL_0].q * 180 / M_PI << ", FL_1:" << std::setprecision(5) << RecvLowROS.motorState[FL_1].q * 180 / M_PI << ", FL_2:" << std::setprecision(5) << RecvLowROS.motorState[FL_2].q * 180 / M_PI << std::endl;
        std::cout << "FR_0:" << std::setprecision(5) << RecvLowROS.motorState[FR_0].q << ", FR_1:" << std::setprecision(5) << RecvLowROS.motorState[FR_1].q << ", FR_2:" << std::setprecision(5) << RecvLowROS.motorState[FR_2].q << std::endl;
        std::cout << "FL_0:" << std::setprecision(5) << RecvLowROS.motorState[FL_0].q << ", FL_1:" << std::setprecision(5) << RecvLowROS.motorState[FL_1].q << ", FL_2:" << std::setprecision(5) << RecvLowROS.motorState[FL_2].q << std::endl;
        //std::cout << "orientation" << ", w:" << std::setprecision(5) << RecvLowROS.imu.quaternion[0] << ", x:" << std::setprecision(5) << RecvLowROS.imu.quaternion[1] << ", y:" << std::setprecision(5) << RecvLowROS.imu.quaternion[2] << ", z:" << std::setprecision(5) << RecvLowROS.imu.quaternion[3]  << std::endl;
        //std::cout << "angular_velocity" << ", x:" << std::setprecision(5) << RecvLowROS.imu.gyroscope[0] << ", y:" << std::setprecision(5) << RecvLowROS.imu.gyroscope[1] << ", z:" << std::setprecision(5) << RecvLowROS.imu.gyroscope[2] << std::endl;
        //std::cout << "linear_acceleration" << ", x:" << std::setprecision(5) << RecvLowROS.imu.accelerometer[0] << ", y:" << std::setprecision(5) << RecvLowROS.imu.accelerometer[1] << ", z:" << std::setprecision(5) << RecvLowROS.imu.accelerometer[2] << std::endl;
        //std::cout << "roll : " << std::setprecision(5) << roll * 180 / M_PI << "/ pitch : " << std::setprecision(5) << pitch * 180 / M_PI << "/ yaw : " << std::setprecision(5) << yaw * 180 / M_PI << std::endl;
        //std::cout << count << std::endl;


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