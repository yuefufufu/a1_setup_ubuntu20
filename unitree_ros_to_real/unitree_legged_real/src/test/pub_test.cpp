#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publisher_test_cpp");
    ros::NodeHandle nh;
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    
    ros::Rate loop_rate(10);

    while (false == ros::isShuttingDown()){
        geometry_msgs::Twist cmd;
        /*int a = 2;
        int b = 2;
        if(a == 1){
            ROS_INFO("a");
        }
        else if(b == 2){
            ROS_INFO("b");
        }*/

        cmd.linear.x = 3.0;
        cmd.angular.z = 0.5;

        cmd_pub.publish(cmd);
        loop_rate.sleep();
    }
    return 0;
}