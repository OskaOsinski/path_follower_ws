#!/bin/bash

export ROS_IP=192.168.0.10
export ROS_MASTER_URI=127.0.0.1

source devel/setup.bash


#clean up step
rosnode kill --all

roslaunch niro_demo_launcher gui_oscc_steering.launch
