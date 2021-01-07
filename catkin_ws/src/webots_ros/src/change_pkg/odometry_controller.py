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
        self.time_step = 16
        self.gyro_values=Vector()
        self.gyro_values_old=Vector()
        self.position=[2,-2.55]
        self.theta=0
        self.motors=Motors_Info()
        self.motors_old=Motors_Info()
        self.WHEEL_RADIUS=0.0985
        self.WHEEL_DISTANCE=0.547
        

    def init(self):
        rospy.init_node(self.node_name, anonymous=True)
        rospy.loginfo('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
        rospy.loginfo('Time step: ' + str(self.time_step))
        self.__get_sensors_values()
        self.start()

    def accelerometer_callback(self, values):
        if abs(values.linear_acceleration.y) > 0.02:
            self.theta -= 0.01*values.linear_acceleration.y*180/math.pi


    def gyro_callback(self, values):
        self.gyro_values_old = self.gyro_values.copy()
        self.gyro_values.x=values.angular_velocity.x
        self.gyro_values.y=values.angular_velocity.y
        self.gyro_values.z=values.angular_velocity.z
        self.gyro_values.t=values.header.stamp    
        if self.gyro_values_old.t != 0:
            prev_ang_vel = 180*self.gyro_values_old.z/math.pi
            ang_vel = 180*self.gyro_values.z/math.pi
            elapsed_time = self.gyro_values.t - self.gyro_values_old.t
            elapsed_time = elapsed_time.to_sec()
            self.theta -= 0.99*((ang_vel+prev_ang_vel)/2 ) *elapsed_time
            self.theta = (self.theta) % 360
            self.theta = self.theta if self.theta <= 180 else self.theta -360
        #utils.debug("x: {:.2f} y: {:.2f} theta: {:.2f}".format(self.position[0], self.position[1], self.theta))        
    
    def update_position(self):
        self.motors_old=self.motors.copy()
        self.motors.right.velocity = utils.call_service(self.robot_name,"wheel_right_joint","get_velocity").value*self.WHEEL_RADIUS #linear velocity
        self.motors.left.velocity  = utils.call_service(self.robot_name,"wheel_left_joint","get_velocity").value*self.WHEEL_RADIUS
        self.motors.time=rospy.Time.now()
        if self.motors_old.time != 0:
            elapsed_time = self.motors.time - self.motors_old.time
            elapsed_time = elapsed_time.to_sec()
            linear_velocity = (self.motors.right.velocity + self.motors.left.velocity) / 2
            # angular_velocity= (self.motors.right.velocity - self.motors.left.velocity) / self.WHEEL_DISTANCE 
            if self.motors.right.velocity * self.motors.left.velocity > 0:
                self.position[0] -= elapsed_time * (linear_velocity * math.sin(self.theta/180*math.pi))
                self.position[1] += elapsed_time * (linear_velocity * math.cos(self.theta/180*math.pi))
            #self.theta -= elapsed_time * angular_velocity
            #self.theta = (self.theta) % 360
            #self.theta = self.theta if self.theta <= 180 else self.theta -360


    def __get_sensor_value(self, topic, device, msg_type):
        try:
            rospy.Subscriber(topic, msg_type, eval("self.%s_callback"%(device)))
        except AttributeError as e:
            utils.logerr(str(e))
        

    def __get_sensors_values(self):
        gyro=False
        accelerometer = False
        while not (gyro and accelerometer): 
            for sensor in rospy.get_published_topics(namespace='/%s'%self.robot_name):
                if 'gyro' in sensor[0] and not gyro:
                    msg_type=globals()[sensor[1].split("/")[1]]
                    topic=sensor[0]
                    device=sensor[0].split("/")[2]
                    self.__get_sensor_value(topic, device, msg_type)
                    gyro=True
                    break
                elif 'accelerometer' in sensor[0] and not accelerometer:
                    msg_type=globals()[sensor[1].split("/")[1]]
                    topic=sensor[0]
                    device=sensor[0].split("/")[2]
                    self.__get_sensor_value(topic, device, msg_type)
                    accelerometer=True    


    def start(self):
        rate = rospy.Rate(10) # 10hz
        while True:
            self.update_position()
            x=self.position[0]
            y=self.position[1]
            theta=self.theta
            utils.publish_odometry(self.topic_name, x,y,theta)
            rate.sleep()

class Motor_Info:
    def __init__(self,velocity=0):
        self.velocity=velocity

    def copy(self):
        return Motor_Info(self.velocity)    

class Motors_Info:
    def __init__(self, left=Motor_Info(), right=Motor_Info(), time=0):
        self.left=left
        self.right=right
        self.time=time

    def copy(self):
        return Motors_Info(self.left.copy(),self.right.copy(),self.time)

