#!/usr/bin/env python
from robot import robot

class Motors:
    def __init__(self):        
        self.left_wheel= RotationalMotor("wheel_left_joint")
        self.right_wheel= RotationalMotor("wheel_right_joint") 
        self.head_horizontal= RotationalMotor("head_1_joint")
        self.head_vertical= RotationalMotor("head_2_joint") 
        self.torso= LinearMotor("torso_lift_joint",0.35,0.07)

    def init(self):
        for key, motor in vars(self).items():
            motor.init()
            

class Motor:
    def __init__(self, name):
        self.name = name

        def set_position(self, position):
            robot.call_service(self.name,'set_position',position)

        def set_velocity(self, velocity):
            robot.call_service(self.name,'set_velocity',velocity)

        def init(self):
            self.set_position(float('inf'))
            self.set_velocity(0.0)

        def __str__(self):
            return name

class LinearMotor(Motor):
    def __init__(self, name, max_height, max_velocity):
        super().__init__(self, name, max_height)
        self.max_height = max_height
        self.max_velocity = max_velocity   

class RotationalMotor(Motor):
    def __init__(self, name):
        super().__init__(self, name)                   


