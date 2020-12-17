#!/bin/bash

cp src/webots_ros/scripts/change_main.py src/webots_ros/scripts/change.py
source devel/setup.bash;
roslaunch webots_ros change.launch;
