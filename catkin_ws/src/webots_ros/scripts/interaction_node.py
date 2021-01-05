#!/usr/bin/env python3
import rospy
from change_pkg.interaction import Interaction

        
def main():
    try:
        if not rospy.is_shutdown():
            interaction = Interaction()
            interaction.init()
            rospy.spin()
    except (rospy.ServiceException, rospy.ROSException):
        pass        


if __name__ == "__main__":  
    main()    

