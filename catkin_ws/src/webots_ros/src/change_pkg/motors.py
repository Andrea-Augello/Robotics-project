#!/usr/bin/env python
import rospy


class Motors:
    def __init__(self,robot):     
        self.left_wheel= RotationalMotor("wheel_left_joint",robot.sensors.left_wheel, robot)
        self.right_wheel= RotationalMotor("wheel_right_joint",robot.sensors.right_wheel, robot) 
        self.head_horizontal= RotationalMotor("head_1_joint",robot.sensors.head_horizontal, robot)
        self.head_vertical= RotationalMotor("head_2_joint",robot.sensors.head_vertical, robot) 
        self.torso= LinearMotor("torso_lift_joint",robot.sensors.torso,robot,0.35,0.07)
        

    def init(self):
        for key, motor in vars(self).items():
            motor.init()
            

class Motor:
    def __init__(self, name, sensor, robot):
        self.name = name
        self.sensor = sensor
        self.robot = robot

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
    def __init__(self, name, sensor, robot, max_height, max_velocity):
        super().__init__(name,sensor, robot)
        self.max_height = max_height
        self.max_velocity = max_velocity   

class RotationalMotor(Motor):
    def __init__(self, name, sensor, robot):
        super().__init__(name, sensor, robot)                   


