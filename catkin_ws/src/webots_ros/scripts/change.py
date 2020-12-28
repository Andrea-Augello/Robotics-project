#!/usr/bin/env python
import rospy
import time
import cv2
import os
from change_pkg.object_recognition import *
from change_pkg.vision import *
from change_pkg.robot import Change

        
def main():
    try:
        if not rospy.is_shutdown():
            robot = Change()
            robot.init()
            robot.controller.start()
            rospy.spin()
    except (rospy.ServiceException, rospy.ROSException):
        pass        


if __name__ == "__main__":  
    main()    

