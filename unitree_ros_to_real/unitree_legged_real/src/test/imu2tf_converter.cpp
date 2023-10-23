#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <sensor_msgs/Imu.h>


void poseCallback(const sensor_msgs::Imu& msg){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    tf::Quaternion q;
    double g = 9.8; //重力加速度
    double r,p; //それぞれroll,pitch。下は、それぞれトピックの値から計算
    r = atan(msg.linear_acceleration.y/msg.linear_acceleration.z);
    p = atan(-msg.linear_acceleration.x/(sqrt(pow(msg.linear_acceleration.y,2)+pow(msg.linear_acceleration.z,2))));
    q.setRPY(r, p, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "lidar_link"));//親フレームをworld,子フレームをcamera_linkとしてTFを配信
}


int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_broadcaster");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/imu/data_raw", 10, &poseCallback);//ここでIMUのトピックを購読

    ros::spin();
    return 0;
};