#!/bin/bash

cd ~/git/Robotics-project/catkin_ws;
source devel/setup.bash;
cd src/webots_ros/scripts;
cat ros_interface.py  > change.py;
cat movement_primitives.py  >> change.py;
cat change_main.py  >> change.py;
roslaunch webots_ros change.launch;
