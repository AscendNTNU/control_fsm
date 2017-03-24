FROM ros:kinetic-ros-base
MAINTAINER Ascend NTNU "www.ascendntnu.no"

ENV ROS_WORKSPACE_PATH=~/catkin_workspace
ENV ROS_PACKAGE_NAME=control_fsm

RUN apt-get update -qq && apt-get install -yqq \
	build-essential


RUN mkdir -p $ROS_WORKSPACE_PATH/src
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_init_workspace $ROS_WORKSPACE_PATH/src'

# Run caktin_make once without building any packages to create the setup.bash
# RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd $ROS_WORKSPACE_PATH; catkin_make'

COPY ./ $ROS_WORKSPACE_PATH/src/$ROS_PACKAGE_NAME/
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd $ROS_WORKSPACE_PATH; catkin_make'
