#!/usr/bin/env python3

from controller import Supervisor
from controller import Robot
import random
import os
import signal
from subprocess import check_output
import math
from random import choice
import matplotlib.pyplot as plt
from numpy.random import random_sample


class Logger (Supervisor):

    def __init__(self):
        self.time_step=1000
        self.sampling_period=2000
        self.robot_height=0.095
        self.path="../../src/change_pkg/robot_position"
        Supervisor.__init__(self)

    def run(self):
        # Init
        trajectory=self.get_trajectory()
        angle_list=self.get_angles()
        total_number_of_pedestrians=20
        min_number_of_pedestrian=5
        number_of_pedestrian=int(random.random()*(total_number_of_pedestrians-min_number_of_pedestrian))+min_number_of_pedestrian
        if not self.step(self.time_step) == -1:
            self.set_pedestrians(number_of_pedestrian,total_number_of_pedestrians,trajectory)
        self.robot = self.getFromDef("ROBOT")
        self.translation = self.robot.getField("translation")
        self.rotation = self.robot.getField("rotation")

        for x,y in trajectory:
            self.translation.setSFVec3f([y,self.robot_height,x])    
            for axis_angle in angle_list:
                if self.step(self.time_step) == -1:
                    quit()  
                #self.rotation.setSFRotation(axis_angle)
                rois=[]
                with open('{}/robot_position.txt'.format(self.path), 'r') as f:
                    for line in f.readlines():
                        roi=self.parse_roi(line)
                        rois.append(roi)
                #TODO to do but Marco will do it                    
            
            

    def parse_roi(self,string_roi):
        #TODO to do
        return string_roi

    def set_pedestrians(self,number_of_pedestrians,total_number_of_pedestrians,trajectory):
        x_room=10
        y_room=10
        pedestrian_list=[]
        for i in range(1,number_of_pedestrians+1):
            x=random.random()*(x_room-1)+0.5-x_room/2
            y=random.random()*(y_room-1)+0.5-y_room/2
            while self.near(x,y,pedestrian_list+trajectory):
                x=random.random()*(x_room-1)+0.5-x_room/2
                y=random.random()*(y_room-1)+0.5-y_room/2
            self.set_pedestrian_position(x,y,i)
        for i in range(number_of_pedestrians+1,total_number_of_pedestrians+1):
            x=x_room+i+1
            y=y_room+i+1
            self.set_pedestrian_position(x,y,i)   

    def set_pedestrian_position(self,x,y,number):
        height=1.27
        name="pedestrian("+str(number)+")"
        node_ref = self.getFromDef(name)
        translation = node_ref.getField("translation")
        translation.setSFVec3f([y,height,x])    
            

    def near(self,x,y,pedestrian_list,tollerance=1):
        for p in pedestrian_list:
            if p[0]-tollerance<x<p[0]+tollerance and p[1]-tollerance<y<p[1]+tollerance:
                return True
        return False

    def close_webots(self):
        pid=int(check_output(["pidof","-s","webots-bin"]))
        os.kill(pid, signal.SIGUSR1)

    def getRandomTrajectory(self):
        num=random.randint(0,2)
        if num == 0:
            return "Square"
        elif num == 1:
            return "Circle"
        elif num == 2:
            return "Random"

    def get_trajectory(self,number_of_points=4):
        trajectory=self.getRandomTrajectory()
        coordinates=[]
        if trajectory == "Square":
            a = 2.5
            b = -2.5

            plt.xlim((b,a))
            plt.ylim((b,a))

            for i in range(number_of_points):
                r = (b - a) * random_sample() + a
                random_point = choice([(choice([a,b]), r),(r, choice([a,b]))])
                coordinates.append((random_point[0],random_point[1]))
        
            return coordinates 
        elif trajectory == "Random":
            for i in range(number_of_points):
                x=random.uniform(-2.5,2.5)
                y=random.uniform(-2.5,2.5)
                coordinates.append((x,y))
                
            return coordinates 
        elif trajectory == "Circle":
            # radius of the circle
            circle_r = 2.5
            # center of the circle (x, y)
            circle_x = 0
            circle_y = 0

            for i in range(number_of_points):
                # random angle
                alpha = 2 * math.pi * random.random()
                x = circle_r * math.cos(alpha) + circle_x
                y = circle_r * math.sin(alpha) + circle_y
                coordinates.append((x,y))
            return coordinates

    def get_angles(self):
        HORIZONTAL_FOV = 57.29578
        return [self.to_axis_angle(i*HORIZONTAL_FOV) for i in range(int(math.ceil(360/HORIZONTAL_FOV)))]

    def to_axis_angle(self,theta):
        #TODO to do
        x=0
        y=0
        z=0
        angle=1
        return (x,y,z,angle)

    
controller = Logger()
controller.run()
