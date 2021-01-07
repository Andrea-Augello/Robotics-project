#!/usr/bin/env python3
import rospy
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

