#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


void cmd_callback(const geometry_msgs::Twist& msg_cmd)
{
        std::cout << msg_cmd.linear.x << std::endl;
}

int main(int argc, char** argv)
{
    //std::cout << "e" << std::endl;
    ros::init(argc, argv, "subscriber_test_cpp");
    ros::NodeHandle nh;
    ros::Subscriber cmd_sub = nh.subscribe("cmd_vel", 10, cmd_callback);
    ros::spin();
    return 0;
}