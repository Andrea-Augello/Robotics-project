#!/usr/bin/env python
import math
from scipy.spatial.transform import Rotation
import numpy as np
import cv2

class Sensors:
    def __init__(self, robot):
        self.lidar =            Sensor("Hokuyo_URG_04LX_UG01",True)
        self.accelerometer =    MovementSensor("accelerometer",True)
        self.base_cover =       Sensor("base_cover_link",False)
        self.sonar_01 =         Sensor("base_sonar_01_link",False)
        self.sonar_02 =         Sensor("base_sonar_02_link",False)
        self.sonar_03 =         Sensor("base_sonar_03_link",False) 
        self.battery =          Sensor("battery_sensor",False)
        self.camera =           Sensor("camera",True)
        self.gyro =             MovementSensor("gyro",True)    
        self.head_horizontal =  Sensor("head_1_joint_sensor",True)
        self.head_vertical =    Sensor("head_2_joint_sensor",True)
        self.inertial_unit =    Sensor("inertial_unit",False)
        self.joystick =         Sensor("joystick",False)
        self.keyboard =         Sensor("keyboard",False)
        self.torso =            Sensor("torso_lift_joint_sensor",True)
        self.left_wheel =       Sensor("wheel_left_joint_sensor",True)
        self.right_wheel =      Sensor("wheel_right_joint_sensor",True)
        self.robot=robot

    def init(self,time_step):
        for key, sensor in vars(self).items():
            if sensor.active and key!='robot':
                sensor.init(time_step)

class Sensor:
    def __init__(self, name, active):                    
        self.name = name
        self.active = active
        self.value = None

    def init(self,time_step):
        self.robot.call_service(self.name,"enable",time_step)

    def inertial_unit_callback(self, values):      
        q = values.orientation
        rot = Rotation.from_quat([q.x,q.y,q.z,q.w])
        rot_euler = rot.as_euler('xyz', degrees=True) 
        self.value = 180*rot_euler[2]/math.pi

    def movement_sensor_callback(self, values):
        self.value.x=values.angular_velocity.x
        self.value.y=values.angular_velocity.y
        self.value.z=values.angular_velocity.z
        self.value.t=values.header.stamp

    def accelerometer_callback(self, values):
        movement_sensor_callback(self, values)

    def gyro_callback(self, values):
        movement_sensor_callback(self, values)

    def camera_callback(self, values):
        image = np.ndarray(shape=(values.height, values.width, 4),
                dtype=np.dtype('uint8'), buffer=values.data)
        size = image.shape
        if size[0] > 1 and size[1] > 1:
            self.value = image.copy()
        else:
            self.value = None
        #cv2.imshow('frame',image)
        #cv2.waitKey(1)

    def motor_sensor_callback(self, values):
        return values.data  

    def head_1_joint_sensor_callback(self, values):
        self.value = motor_sensor_callback(values)

    def head_2_joint_sensor_callback(self, values):
        self.value = motor_sensor_callback(values)   

    def wheel_left_joint_sensor_callback(self, values):
        self.value = motor_sensor_callback(values)
        
    def wheel_right_joint_sensor_callback(self, values):
        self.value = motor_sensor_callback(values)

    def torso_lift_joint_sensor_callback(self, values):
        self.value = motor_sensor_callback(values) 

    def Hokuyo_URG_04LX_UG01_callback(self, values):
        pass


    

class MovementSensor(Sensor):
    def __init__(self, name, active):
        super().__init__(name, active)
        self.value=Vector()


class Vector:
    def __init__(self):
        self.x=0
        self.y=0
        self.z=0
        self.t=0

    def __str__(self):
        return "x:{} y:{} z:{} t:{}".format(self.x,self.y,self.z,self.t)                

