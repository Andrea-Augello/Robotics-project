#!/usr/bin/env python
import rospy


class Motors:
    def __init__(self,robot):
        self.robot=robot      
        self.left_wheel= RotationalMotor("wheel_left_joint",self.robot.sensors.left_wheel)
        self.right_wheel= RotationalMotor("wheel_right_joint",self.robot.sensors.right_wheel) 
        self.head_horizontal= RotationalMotor("head_1_joint",self.robot.sensors.head_horizontal)
        self.head_vertical= RotationalMotor("head_2_joint",self.robot.sensors.head_vertical) 
        self.torso= LinearMotor("torso_lift_joint",self.robot.sensors.torso,0.35,0.07)
        

    def init(self):
        for key, motor in vars(self).items():
            if key!='robot':
                motor.init()
            

class Motor:
    def __init__(self, name, sensor):
        self.name = name
        self.sensor = sensor

    def set_position(self, position):
        self.robot.call_service(self.name,'set_position',position)


    def set_velocity(self, velocity):
        self.robot.call_service(self.name,'set_velocity',velocity)

    def init(self):
        self.set_position(float('inf'))
        self.set_velocity(0.0)


    def __str__(self):
        return name

class LinearMotor(Motor):
    def __init__(self, name, sensor, max_height, max_velocity):
        super().__init__(name,sensor)
        self.max_height = max_height
        self.max_velocity = max_velocity   

class RotationalMotor(Motor):
    def __init__(self, name, sensor):
        super().__init__(name, sensor)                   


