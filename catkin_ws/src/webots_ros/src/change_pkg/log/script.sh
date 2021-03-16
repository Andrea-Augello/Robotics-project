#!/bin/bash
cd ~/git/Robotics-project/catkin_ws;
source devel/setup.bash;
for i in {0..100}
do
    echo $i;
	roslaunch webots_ros change.launch;
done

