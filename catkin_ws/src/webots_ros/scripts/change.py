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
    robot.set_height(robot.motors.torso.max_height/2)
    for i in range(40):
        robot.movement.move_forward(10)
        robot.movement.rotate(90)
        #robot.odometry.movement_history()
    #robot.movement.scan()
    #robot.vision.locate_targets()
    robot.movement.move_forward(2)
    robot.movement.rotate(180)
    robot.movement.move_forward(2)
    robot.movement.rotate(90)
    robot.odometry.movement_history()
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

