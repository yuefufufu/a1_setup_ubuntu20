#include <ros/ros.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>

sensor_msgs::Joy joy_cmd;
double c;

void chatterCallback(const sensor_msgs::Joy& joy_msgs)
{
  //std::cout << joy_msgs << std::endl;
	joy_cmd = joy_msgs;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_master");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  ros::Subscriber joy_sub = nh.subscribe("/joy", 10, chatterCallback);
  ros::Publisher joy_pub = nh.advertise<sensor_msgs::Joy>("/joy/master", 10);
  //joy_cmd.buttons[4] = 0;

  while(ros::ok()){
    /*if(joy_cmd.buttons[4] == 1){
      joy_pub.publish(joy_cmd);
    }*/
    //c = c + 1;
    //ROS_INFO("---------------------------------");
    //std::cout << c << std::endl;
    joy_pub.publish(joy_cmd);
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}