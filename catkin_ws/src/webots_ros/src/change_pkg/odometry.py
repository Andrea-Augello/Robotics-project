#!/usr/bin/env python
import rospy
import math


class Odometry:
    def __init__(self):
        self.x=0
        self.y=0
        self.theta=0

    def update_position(self, distance):
        self.x = self.x + distance[0]*math.cos(self.theta)
        self.y = self.y + distance[1]*math.sin(self.theta)

    def update_theta(self, theta):
        self.theta = (self.theta + theta) % 360      


