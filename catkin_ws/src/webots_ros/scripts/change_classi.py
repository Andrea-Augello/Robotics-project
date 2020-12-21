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
    robot.vision.scan()
    robot.movement.move_forward_accel(2,0.01)
    robot.movement.rotate(180,0.01)
    robot.set_height(robot.motors.torso.max_height)
    robot.set_height(0)
    robot.tablet.speaker.speak_polyglot(it_IT="Ciao sono ciangà e sugnu troppu fuoitti", en_UK="Hello I'm ciangà and I'm too strong")
  
    '''
    scan()
    get_rois(get_current_frames())
    #rotate(90,1)
    for i in range(4):
        move_forward(2,0.05)
        rotate(180,1)
    '''
        
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

