#!/usr/bin/env python
import rospy
import math

robot = None

class Odometry:
    def __init__(self, r):
        global robot
        robot=r
        self.x=0
        self.y=0
        self.theta=0

    def update_position(self, distance):
        self.x = self.x + distance*math.cos(self.theta)
        self.y = self.y + distance*math.sin(self.theta)

    def update_theta(self, theta):
        self.theta = (self.theta + theta) % 360      


