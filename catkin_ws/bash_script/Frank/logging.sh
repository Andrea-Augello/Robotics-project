#!/bin/bash
for i in {1..100}
do
    echo $i
    webots --mode=realtime --batch ~/git/Robotics-project/catkin_ws/src/webots_ros/worlds/change_benchmark_offline.wbt
done

