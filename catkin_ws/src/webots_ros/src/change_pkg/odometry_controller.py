#!/usr/bin/env python
import change_pkg.utils as utils
import rospy
import os
from change_pkg.sensors import Vector
from sensor_msgs.msg import *
from webots_ros.msg import *


class Odometry:
    def __init__(self):
        self.robot_name = 'change'
        self.node_name = 'odometry_node'
        self.topic_name = 'odom'
        self.time_step = 32
        self.gyro_values=Vector()
        self.accelerometer_values=Vector()

    def init(self):
        rospy.init_node(self.node_name, anonymous=True)
        rospy.loginfo('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
        rospy.loginfo('Time step: ' + str(self.time_step))
        self.__get_sensors_values()
        self.start()

    def accelerometer_callback(self, values):
        self.accelerometer_values.x=values.linear_acceleration.x
        self.accelerometer_values.y=values.linear_acceleration.y
        self.accelerometer_values.z=values.linear_acceleration.z
        self.accelerometer_values.t=values.header.stamp

    def gyro_callback(self, values):
        self.gyro_values.x=values.angular_velocity.x
        self.gyro_values.y=values.angular_velocity.y
        self.gyro_values.z=values.angular_velocity.z
        self.gyro_values.t=values.header.stamp    

    def __get_sensor_value(self, topic, device, msg_type):
        try:
            return rospy.Subscriber(topic, msg_type, eval("self.%s_callback"%(device)))
        except AttributeError as e:
            utils.logerr(str(e))
        

    def __get_sensors_values(self):
        sensor_values_count=2
        while sensor_values_count:
            for sensor in rospy.get_published_topics(namespace='/%s'%self.robot_name):
                if 'gyro' in sensor[0] or 'accelerometer' in sensor[0]:
                    msg_type=globals()[sensor[1].split("/")[1]]
                    topic=sensor[0]
                    device=sensor[0].split("/")[2]
                    self.__get_sensor_value(topic, device, msg_type)
                    sensor_values_count-=1

    def start(self):
        while True:
            x=self.gyro_values.x
            y=self.accelerometer_values.y
            theta=0
            utils.publish_odometry(self.topic_name, x,y,theta)
                
