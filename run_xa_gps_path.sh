#!/bin/bash

#export ROS_IP=localhost
#export ROS_MASTER_URI=http://localhost:11311

source devel/setup.bash


#clean up step
#rosnode kill --all

roslaunch niro_demo_launcher gps_navigation_kia.launch runFollower:=1 runVisual:=1

