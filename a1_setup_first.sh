#!/bin/bash

cd ~/a1_setup_ubuntu20
mv lcm ~/
mv unitree_legged_sdk ~/

cd ~/lcm
mkdir build
cd build
cmake ../
make
sudo make install
cd ~/unitree_legged_sdk
mkdir build
cd build
cmake ../
make

cd ~/a1_setup_ubuntu20
mv unitree_ros ~/catkin_ws/src/
mv unitree_ros_to_real ~/catkin_ws/src/
mv unitree_guide ~/catkin_ws/src/

cd
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'export ROS_PACKAGE_PATH=~/catkin_ws:${ROS_PACKAGE_PATH}' >> ~/.bashrc
echo 'export GAZEBO_PLUGIN_PATH=~/catkin_ws/devel/lib:${GAZEBO_PLUGIN_PATH}' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=~/catkin_ws/devel/lib:${LD_LIBRARY_PATH}' >> ~/.bashrc
echo '# 3_1, 3_2' >> ~/.bashrc
echo 'export UNITREE_SDK_VERSION=3_2' >> ~/.bashrc
echo 'export UNITREE_LEGGED_SDK_PATH=~/unitree_legged_sdk' >> ~/.bashrc
echo '#export ALIENGO_SDK_PATH=~/aliengo_sdk' >> ~/.bashrc
echo '# amd64, arm32, arm64' >> ~/.bashrc
echo 'export UNITREE_PLATFORM="amd64"' >> ~/.bashrc
echo 'export LCM_PATH=/usr/local/lib/liblcm.so.1' >> ~/.bashrc

sudo apt-get install -y ros-noetic-controller-interface ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller
sudo apt-get install ros-noetic-teleop-twist-keyboard
sudo apt install ros-noetic-move-base-msgs
sudo apt install liblcm-dev

sudo apt install -y net-tools
sudo apt install -y ipmiutil



reboot
