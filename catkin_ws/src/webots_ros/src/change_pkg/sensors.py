#!/usr/bin/env python
import math
from scipy.spatial.transform import Rotation
import numpy as np
import cv2

robot = None

class Sensors:
    def __init__(self, r):
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
        global robot
        robot=r

    def init(self,time_step):
        for key, sensor in vars(self).items():
            if sensor.active:
                sensor.init(time_step)

class Sensor:
    def __init__(self, name, active):                    
        self.name = name
        self.active = active
        self.value = None

    def init(self,time_step):
        robot.call_service(self.name,"enable",time_step)

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

def inertial_unit_callback(values):
    global robot       
    q = values.orientation
    rot = Rotation.from_quat([q.x,q.y,q.z,q.w])
    rot_euler = rot.as_euler('xyz', degrees=True) 
    robot.sensors.inertial_unit.value = 180*rot_euler[2]/math.pi

def accelerometer_callback(values):
    global robot
    robot.sensors.accelerometer.value.x=values.angular_velocity.x
    robot.sensors.accelerometer.value.y=values.angular_velocity.y
    robot.sensors.accelerometer.value.z=values.angular_velocity.z
    robot.sensors.accelerometer.value.t=values.header.stamp 

def gyro_callback(values):
    global robot
    robot.sensors.gyro.value.x=values.angular_velocity.x
    robot.sensors.gyro.value.y=values.angular_velocity.y
    robot.sensors.gyro.value.z=values.angular_velocity.z
    robot.sensors.gyro.value.t=values.header.stamp 

def camera_callback(values):
    global robot
    image = np.ndarray(shape=(values.height, values.width, 4),
            dtype=np.dtype('uint8'), buffer=values.data)
    size = image.shape
    if size[0] > 1 and size[1] > 1:
        robot.sensors.camera.value = image.copy()
    else:
        robot.sensors.camera.value = None
    #cv2.imshow('frame',image)
    #cv2.waitKey(1)

def motor_sensor_callback(values):
    return values.data  

def head_1_joint_sensor_callback(values):
    global robot
    robot.sensors.head_horizontal.value = motor_sensor_callback(values)

def head_2_joint_sensor_callback(values):
    global robot
    robot.sensors.head_vertical.value = motor_sensor_callback(values)   

def wheel_left_joint_sensor_callback(values):
    global robot
    robot.sensors.left_wheel.value = motor_sensor_callback(values)
    
def wheel_right_joint_sensor_callback(values):
    global robot
    robot.sensors.right_wheel.value = motor_sensor_callback(values)

def torso_lift_joint_sensor_callback(values):
    global robot
    robot.sensors.torso.value = motor_sensor_callback(values) 

def Hokuyo_URG_04LX_UG01_callback(values):
    pass


