#!/usr/bin/env python

from controller import Supervisor
from controller import Robot
import random
import os
import signal
from subprocess import check_output
import math


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
            
            

    def parse_roi(self,string_roi):
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

    def get_trajectory(self):
        return [(2.5,-2.5),(2.5,2.5),(-2.5,2.5),(-2.5,-2.5)]   

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
