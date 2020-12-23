#!/usr/bin/env python
import rospy
import math
import matplotlib.pyplot as plt


class Odometry:
    def __init__(self):
        self.history=[[0,0]]
        self.x=0
        self.y=0
        self.theta=0

    def update_position(self, distance):
        self.x = self.x + distance[1]*math.cos(math.pi*self.theta/180) - distance[0]*math.sin(math.pi*self.theta/180)
        self.y = self.y + distance[1]*math.sin(math.pi*self.theta/180) + distance[0]*math.cos(math.pi*self.theta/180)
        self.history.append([self.x, self.y])

    def update_theta(self, theta):
        self.theta = (self.theta + theta) % 360
        self.theta = self.theta if self.theta <= 180 else self.theta -360

    def movement_history(self):
        plt.plot([x for [x,y] in self.history], [y for [x,y] in self.history]) 
        plt.grid(True)
        plt.show()    

    def __str__(self):
        return "x:{} y:{} theta:{}".format(self.x,self.y,self.theta)          


