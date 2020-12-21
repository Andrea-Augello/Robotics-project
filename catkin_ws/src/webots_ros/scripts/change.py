#!/usr/bin/env python
import rospy
import time
import cv2
import os
from change_pkg.object_recognition import *
from change_pkg.vision import *
from change_pkg.robot import Change

def testing(robot):
    robot.tablet.display.load_image('warning')
    robot.movement.scan()
    robot.vision.locate_targets()
    rospy.logerr(robot.odometry)
    robot.movement.move_forward(2)
    robot.movement.rotate(180)
    rospy.logerr(robot.odometry)
    robot.set_height(robot.motors.torso.max_height)
    robot.set_height(0)
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

