#!/bin/bash


sudo ip link set down can0
sudo ip link set down can1
sudo ip link set down can2
sudo ip link set down can3

sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
sudo ip link set can1 type can bitrate 500000
sudo ip link set up can1
sudo ip link set can2 type can bitrate 500000
sudo ip link set up can2
sudo ip link set can3 type can bitrate 500000
sudo ip link set up can3
