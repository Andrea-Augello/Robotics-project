#!/usr/bin/env python
import rospy

robot=None

class Motors:
    def __init__(self,r):
        global robot
        robot=r        
        self.left_wheel= RotationalMotor("wheel_left_joint",r.sensors.left_wheel)
        self.right_wheel= RotationalMotor("wheel_right_joint",r.sensors.right_wheel) 
        self.head_horizontal= RotationalMotor("head_1_joint",r.sensors.head_horizontal)
        self.head_vertical= RotationalMotor("head_2_joint",r.sensors.head_vertical) 
        self.torso= LinearMotor("torso_lift_joint",r.sensors.torso,0.35,0.07)
        

    def init(self):
        for key, motor in vars(self).items():
            motor.init()
            

class Motor:
    def __init__(self, name, sensor):
        self.name = name
        self.sensor = sensor
    

    @property
    def position(self):
        return self.__position   

    @position.setter
    def position(self, position):
        global robot
        robot.call_service(self.name,'set_position',position)
        self.__position=position

    @property
    def velocity(self):
        return self.__velocity    

    @velocity.setter
    def velocity(self, velocity):
        global robot
        robot.call_service(self.name,'set_velocity',velocity)
        self.__velocity=velocity

    def init(self):
        self.position=float('inf')
        #robot.call_service(self.name,'set_position',float('inf'))
        self.velocity=0.0
        #robot.call_service(self.name,'set_velocity',0.0)

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


