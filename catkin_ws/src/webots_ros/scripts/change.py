#!/usr/bin/env python
import rospy
import time
import cv2
import os
from change_pkg.object_recognition import *
from change_pkg.vision import *
from change_pkg.robot import Change

def testing(robot):
    robot.tablet.warning()
    valid_target = False
    bugmode = 0
    distances = [0,0,0,0]
    while not valid_target:
        robot.movement.scan()
        valid_target = robot.path_planner.set_target(robot.path_planner.find_clusters(robot.vision.locate_targets()))
    while robot.path_planner.target_distance() > 0.5:
        robot.print_info()
        if bugmode > 1:
            distance,angle = robot.path_planner.bug_next_step()
            robot.movement.rotate(angle)
            robot.movement.move_forward(distance)
            bugmode = bugmode - 1
        else :
            angle = robot.path_planner.next_step_direction()
            robot.movement.rotate(angle)
            distance=robot.path_planner.movement_distance()
            robot.movement.move_forward(distance)
            distances.pop(0)
            distances.append(robot.odometry.distance_traveled)
            if distances[3] - distances[0] < 0.1:
                bugmode = 3
    robot.movement.rotate(-robot.path_planner.target_angle())
    robot.print_info()    
    robot.odometry.movement_history()
    robot.tablet.speaker.speak_polyglot(it_IT="Ciao sono ciangà e sugnu troppu fuoitti", en_UK="Hello I'm ciangà and I'm too strong")

  
        
def main():
    try:
        if not rospy.is_shutdown():
            robot = Change()
            robot.init()
            testing(robot)
            rospy.spin()
    except (rospy.ServiceException, rospy.ROSException):
        pass        

                

if __name__ == "__main__":  
    main()    

