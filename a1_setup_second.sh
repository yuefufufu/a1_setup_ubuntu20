#!/bin/bash

cd ~/catkin_ws
catkin build unitree_legged_msgs
catkin build

roslaunch unitree_gazebo normal.launch rname:=a1 wname:=earth
