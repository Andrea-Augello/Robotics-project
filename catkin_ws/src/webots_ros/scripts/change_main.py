#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from webots_ros.srv import *
from movement_primitives import *
from ros_interface import *
import os
import rosservice



def testing():
    #set_linear_velocity(3.0)
    load_image('warning')
    call_service('speaker', 'set_language', 'it-IT')
    speak("Ciao sono ciangà e sugnu troppu fuoitti")
    speak_polyglot(it_IT="ciao", en_UK="Hello")
    rospy.logerr(rospy.get_published_topics(namespace='/%s'%model_name))
	

def main():
    if not rospy.is_shutdown():
        rospy.init_node(model_name, anonymous=True)
        rospy.loginfo('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
        rospy.loginfo('Time step: ' + str(time_step))
        motor_init()
        enable_sensors()
        get_sensor_values()
        testing()
        rospy.spin()

                

if __name__ == "__main__":  
    main()    

