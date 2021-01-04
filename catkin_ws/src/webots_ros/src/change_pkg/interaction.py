#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import change_pkg.utils as utils
import os
import rosservice

class Interaction:
    def __init__(self):
        self.name = 'interaction_node'
        self.speaker_name='speaker'
        self.time_step = 32

    def init(self):
        rospy.init_node(self.name, anonymous=True)
        rospy.loginfo('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
        rospy.loginfo('Time step: ' + str(self.time_step))
        rospy.Subscriber("/"+self.name+"/speaker", String, self.speaker_callback)

    def speaker_callback(self, values):
        if values.data != 'enable':
            for message in values.data.split('|'):
                [language, text] = message.split('@')
                self.call_robot_service(self.speaker_name, 'set_language', language)
                self.speak(text)


    def is_speaking(self):
        response = self.call_robot_service(self.speaker_name,'is_speaking')
        return response.value

    def speak(self,text,volume=1.0):
        while(self.is_speaking()):
            pass
        self.call_robot_service(self.speaker_name,'speak', text, volume)


    def call_robot_service(self,device_name,service_name,*args):
        robot_name='change'
        service_string = "/%s/%s/%s" % (robot_name, device_name, service_name)
        rospy.loginfo(service_string)
        rospy.wait_for_service(service_string)
        try:
            service = rospy.ServiceProxy(service_string,rosservice.get_service_class_by_name(service_string))
            response = service(*args)
            rospy.loginfo("Service %s called" % service_string)
            return response
        except rospy.ServiceException as e:
            utils.logerr("Service call failed: %s" % e)                     
