#!/usr/bin/env python

class Motors:
    def __init__(self,robot):     
        self.left_wheel= RotationalMotor("wheel_left_joint",robot.sensors.left_wheel, robot)
        self.right_wheel= RotationalMotor("wheel_right_joint",robot.sensors.right_wheel, robot) 
        self.head_horizontal= RotationalMotor("head_1_joint",robot.sensors.head_horizontal, robot,-1.24,1.24)
        self.head_vertical= RotationalMotor("head_2_joint",robot.sensors.head_vertical, robot,-0.98,0.79) 
        self.torso= LinearMotor("torso_lift_joint",robot.sensors.torso,robot,0.35,0.07)
        

    def init(self):
        for key, motor in vars(self).items():
            motor.init()
            

class Motor:
    def __init__(self, name, sensor, robot):
        self.name = name
        self.sensor = sensor
        self.__robot = robot

    def set_position(self, position):
        self.__robot.call_service(self.name,'set_position',position)


    def set_velocity(self, velocity):
        self.__robot.call_service(self.name,'set_velocity',velocity)

    def init(self):
        self.set_position(float('inf'))
        self.set_velocity(0.0)


    def __str__(self):
        return name

class LinearMotor(Motor):
    def __init__(self, name, sensor, robot, max_height, max_velocity):
        super().__init__(name,sensor, robot)
        self.max_height = max_height
        self.max_velocity = max_velocity   

class RotationalMotor(Motor):
    def __init__(self, name, sensor, robot, min_position=None, max_position=None):
        super().__init__(name, sensor, robot)
        self.min_position=min_position
        self.max_position=max_position                   


