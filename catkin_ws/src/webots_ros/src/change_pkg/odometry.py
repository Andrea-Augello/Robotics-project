#!/usr/bin/env python
import math
import numpy as np
import matplotlib.pyplot as plt


class Odometry:
    def __init__(self):
        self.x=2
        self.y=-2.55
        self.history=[((self.x,self.y),0)]
        self.theta=0

    def get_position(self):
        return (self.x,self.y)    

    def update_position(self, distance):
        # TODO check if utils.math_distance is better then np.hypot
        distance_traveled = self.history[0][1] + np.hypot(distance[0], distance[1])
        self.x = self.x + distance[1]*math.cos(math.pi*self.theta/180) - distance[0]*math.sin(math.pi*self.theta/180)
        self.y = self.y + distance[1]*math.sin(math.pi*self.theta/180) + distance[0]*math.cos(math.pi*self.theta/180)
        self.history.insert(0,((self.x, self.y),distance_traveled))

    def update_theta(self, theta):
        self.theta = (self.theta + theta) % 360
        self.theta = self.theta if self.theta <= 180 else self.theta -360

    def movement_history(self):
        fig, ax = plt.subplots()
        fig.suptitle('Movement history') 
        ax.plot([x for ((x,y),_distance) in self.history], [y for ((x,y),_distance) in self.history])
        ax.axis("equal")
        ax.set_xlabel('Position (m)')
        ax.set_ylabel('Position (m)')
        ax.invert_xaxis() 
        ax.grid(True)
        plt.show()    

    def abs_cartesian_to_polar(self, p):
        """Accepts a point given in cartesian coordinates relative to the world
        frame of reference and converts it to polar coordinates in the robot
        frame of reference based on the current position estimate

        :p:   Cartesian coordinates in the (x, y) format
        :returns:  Cartesian coordinates in the (Rho, Theta) format

        """
        x = p[0] - self.x
        y = p[1] - self.y
        angle = -180/math.pi*math.atan2(x,y)-self.theta
        angle = angle if angle < 180 else angle -360
        return (np.hypot(x,y), angle)


    def polar_to_abs_cartesian(self, p):
        """Accepts a point given in polar coordinates relative to the robot
        frame of reference and converts it to cartesian coordinates in a wolrd
        frame of reference based on the current position estimate

        :p:   Polar coordinates in the (Rho, Theta) format
        :returns:       Cartesian coordinates

        """
        angle=p[1]+self.theta
        return (self.x+p[0]*math.sin(math.pi*angle/180),
                self.y+p[0]*math.cos(math.pi*angle/180))


    def __str__(self):
        return "x:{:.2f} y:{:.2f} theta:{:.2f}".format(self.x,self.y,self.theta)  


