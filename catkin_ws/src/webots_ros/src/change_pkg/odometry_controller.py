#!/usr/bin/env python
import change_pkg.utils as utils
import rospy
import math
import os
from change_pkg.sensors import Vector
from sensor_msgs.msg import *


class Odometry:
    def __init__(self):
        self.robot_name = 'change'
        self.node_name = 'odometry_node'
        self.topic_name = 'odom'
        self.time_step = 32
        self.gyro_values=Vector()
        self.gyro_values_old=Vector()
        self.accelerometer_values=Vector()
        self.accelerometer_values_old=Vector()
        self.speed=[0,0]
        self.speed_old=[0,0]
        self.pos=[2,-2.55]
        self.theta=0

    def init(self):
        rospy.init_node(self.node_name, anonymous=True)
        rospy.loginfo('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
        rospy.loginfo('Time step: ' + str(self.time_step))
        self.__get_sensors_values()
        self.start()

    def accelerometer_callback(self, values):
        self.accelerometer_values_old = self.accelerometer_values
        ax =  values.linear_acceleration.y*math.cos(math.pi*self.theta/180) \
                - values.linear_acceleration.x*math.sin(math.pi*self.theta/180)
        ay =  values.linear_acceleration.y*math.sin(math.pi*self.theta/180) \
                + values.linear_acceleration.x*math.cos(math.pi*self.theta/180)

        self.accelerometer_values.x=ax
        self.accelerometer_values.y=ay
        self.accelerometer_values.z=values.linear_acceleration.z
        self.accelerometer_values.t=values.header.stamp
        if self.accelerometer_values_old.t != 0:
            timestamp = self.accelerometer_values.t
            accel = [self.accelerometer_values.x, self.accelerometer_values.y]
            prev_accel = [self.accelerometer_values_old.x, self.accelerometer_values_old.y]
            elapsed_time = timestamp-self.accelerometer_values_old.t
            elapsed_time = elapsed_time.to_sec()
            for i in range(2):
                self.speed[i] +=((accel[i]+prev_accel[i])/2)*elapsed_time
                self.pos[i] += ((self.speed[i]+self.speed_old[i])/2 ) *elapsed_time
            self.speed_old = self.speed


    def gyro_callback(self, values):
        self.gyro_values_old = self.gyro_values
        self.gyro_values.x=values.angular_velocity.x
        self.gyro_values.y=values.angular_velocity.y
        self.gyro_values.z=values.angular_velocity.z
        self.gyro_values.t=values.header.stamp    
        if self.gyro_values_old.t != 0:
            prev_ang_vel = 180*self.gyro_values_old.z/math.pi
            ang_vel = 180*self.gyro_values.z/math.pi
            elapsed_time = self.gyro_values.t - self.gyro_values_old.t
            elapsed_time = elapsed_time.to_sec()
            self.theta -= ((ang_vel+prev_ang_vel)/2 ) *elapsed_time
            self.theta = (self.theta) % 360
            self.theta = self.theta if self.theta <= 180 else self.theta -360

    def __get_sensor_value(self, topic, device, msg_type):
        try:
            rospy.Subscriber(topic, msg_type, eval("self.%s_callback"%(device)))
        except AttributeError as e:
            utils.logerr(str(e))
        

    def __get_sensors_values(self):
        gyro=False
        accelerometer=False
        while not gyro or not accelerometer:
            for sensor in rospy.get_published_topics(namespace='/%s'%self.robot_name):
                if 'gyro' in sensor[0] and not gyro:
                    msg_type=globals()[sensor[1].split("/")[1]]
                    topic=sensor[0]
                    device=sensor[0].split("/")[2]
                    self.__get_sensor_value(topic, device, msg_type)
                    gyro=True
                elif 'accelerometer' in sensor[0] and not accelerometer:
                    msg_type=globals()[sensor[1].split("/")[1]]
                    topic=sensor[0]
                    device=sensor[0].split("/")[2]
                    self.__get_sensor_value(topic, device, msg_type)
                    accelerometer=True    


    def start(self):
        rate = rospy.Rate(10) # 10hz
        while True:
            x=self.pos[0]
            y=self.pos[1]
            theta=self.theta
            utils.publish_odometry(self.topic_name, x,y,theta)
            rate.sleep()
                
