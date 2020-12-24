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
    robot.path_planner.set_target((-3,7))
    while robot.path_planner.target_distance() > 0.1:
        robot.print_info()
        angle = robot.path_planner.next_step_direction()
        robot.movement.rotate(angle)
        distance=robot.path_planner.movement_distance()
        robot.movement.move_forward(distance)
    robot.print_info()    
    #robot.movement.scan()
    #robot.vision.locate_targets()
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

