import ast
from controller import Supervisor
from controller import Robot
import random
import os
import signal
from subprocess import check_output
from random import choice
from numpy.random import random_sample
import numpy as np
import math
import time



class Logger (Supervisor):

    def __init__(self):
        self.time_step=1000
        self.sampling_period=2000
        self.robot_height=0.095
        self.path="../../src/change_pkg/robot_position"
        Supervisor.__init__(self)

    def run(self):
        t="Rectangular"
        self.trajectory=self.get_trajectory(number_of_points=8,trajectory=t)
        with open("{}/trajectory.txt".format(self.path),"w") as f:
            f.write(str(self.trajectory))
        total_number_of_pedestrians=20
        max_number_of_pedestrian=10
        min_number_of_pedestrian=4
        number_of_pedestrian=int(random.random()*(max_number_of_pedestrian-min_number_of_pedestrian))+min_number_of_pedestrian
        if not self.step(self.time_step) == -1:
            self.pedestrian_list=self.set_pedestrians(number_of_pedestrian,total_number_of_pedestrians)
        self.robot = self.getFromDef("ROBOT")
        self.translation = self.robot.getField("translation")


        with open('{}/robot_position_{}.txt'.format(self.path,len(self.trajectory)), 'w') as f:
            f.write("0,-2.5,2.5\n")
            f.close()
        while not self.step(self.time_step) == -1:
            with open('{}/robot_position_{}.txt'.format(self.path,len(self.trajectory)), 'r') as f:
                [flag,x,y]=f.readline().split(",")
            if int(flag):    
                self.root_node_ref = self.getFromDef("ROBOT")
                self.root_translation_field = self.root_node_ref.getField("translation")
                self.root_translation_field.setSFVec3f([float(x),0.095,float(y)])
                f.close()
                with open('{}/robot_position_{}.txt'.format(self.path,len(self.trajectory)), 'w') as f:
                    f.write("0,{},{}\n".format(x,y))
                    f.close()    
            


    def set_pedestrians(self,number_of_pedestrians,total_number_of_pedestrians):
        x_room=10
        y_room=10
        pedestrian_list=[]
        for i in range(1,number_of_pedestrians+1):
            x=random.random()*(x_room-1)+0.5-x_room/2
            y=random.random()*(y_room-1)+0.5-y_room/2
            while self.near(x,y,pedestrian_list+self.trajectory):
                x=random.random()*(x_room-1)+0.5-x_room/2
                y=random.random()*(y_room-1)+0.5-y_room/2
            self.set_pedestrian_position(x,y,i)
            pedestrian_list.append((x,y))
        for i in range(number_of_pedestrians+1,total_number_of_pedestrians+1):
            x=x_room+i+1
            y=y_room+i+1
            self.set_pedestrian_position(x,y,i)
        return pedestrian_list       

    def set_pedestrian_position(self,x,y,number):
        height=1.27
        name="pedestrian("+str(number)+")"
        node_ref = self.getFromDef(name)
        translation = node_ref.getField("translation")
        translation.setSFVec3f([x,height,y])    
            

    def near(self,x,y,pedestrian_list,tollerance=1):
        for p in pedestrian_list:
            if p[0]-tollerance<x<p[0]+tollerance and p[1]-tollerance<y<p[1]+tollerance:
                return True
        return False

    def getRandomTrajectory(self):
        trajectories=["Rectangular","Circle","Random"]
        return random.choice(trajectories)

    def get_trajectory(self,x=2.5,y=-2.5,number_of_points=4,trajectory=None):
        #TODO to fix square case
        if trajectory==None:
            trajectory=self.getRandomTrajectory()
        coordinates=[]
        if trajectory == "Rectangular":
            #For starting bottom-right
            x=abs(x)
            y=-abs(y)  
            h=2*x
            w=2*abs(y)
            perimeter=2*(h+w)
            step=perimeter/number_of_points
            x_current=x
            y_current=y

            while y_current<=-y:
                coordinates.append((y_current,-x_current))
                y_current+=step
            x_current-=y_current+y
            y_current=abs(y)    
            while x_current>=-x:
                coordinates.append((y_current,-x_current))
                x_current-=step
            y_current+=x_current+x
            x_current=-x
            while y_current>=y:
                coordinates.append((y_current,-x_current))
                y_current-=step
            x_current-=y_current-y
            y_current=y
            while x_current<x:
                coordinates.append((y_current,-x_current))
                x_current+=step        

            return coordinates 
        elif trajectory == "Random":
            x_room=10
            y_room=10
            for _ in range(number_of_points):
                x=random.random()*(x_room-1)+0.5-x_room/2
                y=random.random()*(y_room-1)+0.5-y_room/2
                coordinates.append((x,y))
            return coordinates 
        elif trajectory == "Circle":
            # radius of the circle
            circle_r = math.hypot(x,y)
            # center of the circle (x, y)
            circle_x = 0
            circle_y = 0

            for _ in range(number_of_points):
                # random angle
                alpha = 2 * math.pi * random.random()
                x = circle_r * math.cos(alpha) + circle_x
                y = circle_r * math.sin(alpha) + circle_y
                coordinates.append((x,y))
            return coordinates

    
controller = Logger()
controller.run()
