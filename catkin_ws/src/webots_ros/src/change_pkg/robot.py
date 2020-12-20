#!/usr/bin/env python
class Change:
    def __init__(self):
        self.name = 'change'
        self.motors = Motors()
        self.sensors = Sensors()

    def __str__(self):
        return name


class Motors:
    def __init__(self):        
        self.left_wheel= Motor("wheel_left_joint")
        self.right_wheel= Motor("wheel_right_joint") 
        self.head_horizontal= Motor("head_1_joint")
        self.head_vertical= Motor("head_2_joint") 
        self.torso= Motor("torso_lift_joint")

class Motor:
    def __init__(self, name):
        self.name = name

        def set_position():
            pass

        def get_position():
            pass

        def set_velocity():
            pass


