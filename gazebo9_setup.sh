#!/bin/bash

cd
sudo apt remove -y ros-noetic-gazebo*
sudo apt remove -y libgazebo*
sudo apt remove -y gazebo*
sudo apt install -y gazebo9*

cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin build

cd
sudo apt-get install -y libgazebo9-dev
cd ~/catkin_ws/src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b noetic-devel
rosdep update
cd ~/catkin_ws
catkin build
cd
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

gazebo -v
