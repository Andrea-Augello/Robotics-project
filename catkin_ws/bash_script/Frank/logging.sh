#!/bin/bash
for i in {1..100}
do
   webots --mode=fast ~/git/Robotics-project/catkin_ws/src/webots_ros/worlds/change.wbt
done

