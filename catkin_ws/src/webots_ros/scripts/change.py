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
    robot.path_planner.set_target([-3,7])
    while   robot.path_planner.target_distance() > 1:
        angle = robot.path_planner.next_step_direction()
        rospy.logerr(angle)
        robot.movement.rotate(angle)
        robot.movement.move_forward(0.5)
    # robot.movement.scan()
    # robot.vision.locate_targets()
    robot.odometry.movement_history()
    robot.set_height(0)
    robot.tablet.speaker.speak_polyglot(it_IT="Ciao sono ciangà e sugnu troppu fuoitti", en_UK="Hello I'm ciangà and I'm too strong")
    robot.print_info()

  
        
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

