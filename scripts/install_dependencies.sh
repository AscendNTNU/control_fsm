#!/bin/bash

ROS_VERSION=$(rosversion -d)
sudo apt-get install ros-$ROS_VERSION-mavros ros-$ROS_VERSION-mavros-extras ros-$ROS_VERSION-vrpn-client-ros ros-$ROS_VERSION-tf2

