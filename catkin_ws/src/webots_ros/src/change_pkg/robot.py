#!/usr/bin/env python
import os
import rospy
import change_pkg.motors as motors
import change_pkg.sensors as sensors
import change_pkg.tablet as tablet
import change_pkg.movement as movement
import change_pkg.vision as vision
import change_pkg.odometry as odometry
from sensor_msgs.msg import *
from webots_ros.msg import *
import rosservice

class Change:
    def __init__(self):
        self.name = 'change'
        self.time_step = 32
        self.wheel_diameter = 0.23
        self.sensors = sensors.Sensors(self)
        self.motors = motors.Motors(self)
        self.tablet = tablet.Tablet(self)
        self.movement = movement.Movement(self)
        self.vision = vision.Vision(self)
        self.odometry = odometry.Odometry(self)

    def __str__(self):
        return self.name

    def init(self):
        rospy.init_node(self.name, anonymous=True)
        rospy.loginfo('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
        rospy.loginfo('Time step: ' + str(self.time_step))
        self.motors.init()
        self.sensors.init(self.time_step)
        self.__get_sensors_values()

    def set_height(self, height):
        if height>=0 and height<=self.motors.torso.max_height:
            self.motors.torso.set_position(height)
            self.motors.torso.set_velocity(self.motors.torso.max_velocity)
            while abs(self.sensors.torso.value - height) > 0.002:
                pass

    
    def __get_sensor_value(self, topic, device, msg_type):
        try:
            return rospy.Subscriber(topic, msg_type, eval("sensors.%s_callback"%device))
        except NameError as e:
            rospy.logerr(str(e))
        

    def __get_sensors_values(self):
        for sensor in rospy.get_published_topics(namespace='/%s'%self.name):
            if 'range_image' not in sensor[0]: 
                msg_type=globals()[sensor[1].split("/")[1]]
                topic=sensor[0]
                device=sensor[0].split("/")[2]
                self.__get_sensor_value(topic, device, msg_type)

    def call_service(self,device_name,service_name,*args):
        service_string = "/%s/%s/%s" % (self.name, device_name, service_name)
        rospy.loginfo(service_string)
        rospy.wait_for_service(service_string)
        try:
            service = rospy.ServiceProxy(service_string,rosservice.get_service_class_by_name(service_string))
            response = service(*args)
            rospy.loginfo("Service %s called" % service_string)
            return response
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)                           



