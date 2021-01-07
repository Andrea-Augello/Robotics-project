#!/bin/bash
export ROS_MASTER_URI=http://Frank-Desktop:11311

cd ~/git/Robotics-project/catkin_ws;
source devel/setup.bash;
roslaunch webots_ros pi.launch;
