#!/usr/bin/env python3
import rospy
from change_pkg.odometry_controller import Odometry

        
def main():
    try:
        if not rospy.is_shutdown():
            odometry = Odometry()
            odometry.init()
            rospy.spin()
    except (rospy.ServiceException, rospy.ROSException):
        pass        


if __name__ == "__main__":  
    main()    

