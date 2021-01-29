#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import change_pkg.utils as utils
import os
import rosservice

class Interaction:
    def __init__(self):
        self.name = 'interaction_node'
        self.speaker_name='speaker'
        self.display_name='display'
        self.path='../../../../../Media/'
        self.is_speaking_flag=False
        self.time_step = 32
        self.slideshow_image_number=3

    def init(self):
        rospy.init_node(self.name, anonymous=True)
        rospy.loginfo('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
        rospy.loginfo('Time step: ' + str(self.time_step))
        rospy.Subscriber("/"+self.name+"/"+self.speaker_name, String, self.speaker_callback)
        rospy.Subscriber("/"+self.name+"/"+self.display_name, String, self.display_callback)
        self.slideshow()

    def speaker_callback(self, values):
        if values.data != 'enable':
            self.is_speaking_flag=True
            for message in values.data.split('|'):
                [language, text] = message.split('@')
                self.call_robot_service(self.speaker_name, 'set_language', language)
                self.speak(text)
            while(self.is_speaking()):
                pass    
            self.is_speaking_flag=False    

    def slideshow(self):
        counter=0
        images=['social_distancing_'+str(i) for i in range(1,self.slideshow_image_number+1)]
        while True:
            if not self.is_speaking_flag:
                self.load_image(images[counter])
                rospy.sleep(5)
                counter=(counter+1)%len(images)                    

    def display_callback(self, values):
        if values.data != 'enable':
            self.load_image(values.data)


    def load_image(self,image):
        image_loaded = self.call_robot_service(self.display_name,'image_load',self.path+'Image/'+image+'.jpg')
        self.call_robot_service(self.display_name,'image_paste',image_loaded.ir,0,0,False)

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
