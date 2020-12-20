#!/usr/bin/env python
import rospy
import time
import cv2
from std_msgs.msg import *
from webots_ros.srv import *
from change_pkg.movement_primitives import *
from change_pkg.ros_interface import *
import os
import rosservice



def testing():
    load_image('warning')
    time.sleep(5)
    #rotate(90,1)
    for i in range(4):
        move_forward(2,0.1)
        rotate(180,1)
    call_service('speaker', 'set_language', 'it-IT')
    speak("Ciao sono ciangà e sugnu troppu fuoitti")
    speak_polyglot(it_IT="ciao", en_UK="Hello")
    
        
def main():
    if not rospy.is_shutdown():
        rospy.init_node(model_name, anonymous=True)
        rospy.loginfo('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
        rospy.loginfo('Time step: ' + str(time_step))
        motor_init()
        enable_sensors()
        get_sensors_values()
        testing()
        rospy.spin()

                

if __name__ == "__main__":  
    main()    

