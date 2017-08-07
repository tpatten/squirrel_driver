#!/bin/bash

export ROS_MASTER_URI=http://10.42.0.1:11311
export ROS_IP=10.42.0.2
source /opt/ros/indigo/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
roslaunch squirrel_interaction squirrel_interaction.launch

