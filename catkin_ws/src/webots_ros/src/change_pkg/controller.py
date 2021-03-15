#!/usr/bin/env python3

import change_pkg.path_planning as path_planner
import change_pkg.utils as utils
import itertools
import numpy as np
import change_pkg.people_density as people_density
import matplotlib.pyplot as plt
import matplotlib.path as mpltPath
import time
import random
import ast
import os
import signal
from subprocess import check_output
#from controller import Supervisor

class Controller:
    def __init__(self, robot):
        self.people_density = people_density.GridMap(robot)
        self.__robot=robot
        self.scheduler=Scheduler()
        self.path_planner = path_planner.Path_planner(robot)
        self.loop_point= None
        self.LOOKBACK_WINDOW_SIZE = 4
        self.MOVEMENT_LOOP_PRECISION = 0.01
        self.ROTATION_LOOP_PRECISION = 0.1
        self.ESCAPING_DISTANCE = 3
        self.TARGET_DISTANCE = 1.5
        self.SCAN_RATE = 10

    def close_webots(self):
        pid=int(check_output(["pidof","-s","webots-bin"]))
        os.kill(pid, signal.SIGUSR1)

    def start(self):
        #supervisor=Supervisor()
        while False:
            for i in range(10):
                self.__robot.movement.move_forward(10-0.5*i)
                self.__robot.movement.rotate(-90)
                self.__robot.print_info()
                self.__robot.odometry.movement_history()
        while False:
            # Sensor characterization
            self.__robot.vision.clear_saved_frames()
            self.__robot.vision.save_frame(self.__robot.sensors.camera.value)
            utils.loginfo(self.__robot.vision.locate_targets())
            time.sleep(3)
        if True:
            # distance estimation
            path="/home/frank/git/Robotics-project/catkin_ws/src/webots_ros/src/change_pkg/robot_position"
            with open("{}/trajectory.txt".format(path),"r") as f:
                lines=f.readlines()
                trajectory=ast.literal_eval(lines[0].strip())
                trajectory_type=lines[1].strip()
                for p in trajectory:
                    self.scan()
                    with open('{}/robot_position_{}_{}.txt'.format(path,trajectory_type,str(len(trajectory))), 'w') as f:
                        f.write("1,{},{}\n".format(p[1],-p[0]))
                        f.write(trajectory_type+"\n")
                        f.close()
                        self.__robot.odometry.x=p[0]
                        self.__robot.odometry.y=p[1]
                        time.sleep(1)
                    #self.__robot.movement.move_forward(5)
                    #self.__robot.movement.rotate(85)
            self.close_webots()     
              
                 
        while False:   
            self.exploration()
            self.go_to_gathering()
            self.__robot.warning()
            if self.__robot.debug_mode:
                self.__robot.odometry.movement_history()

    def go_to_gathering(self):
        self.scheduler.potential_field_mode()
        odometer = self.get_odometer()
        while not self.is_arrived():
            #self.people_density.reset()
            if self.get_odometer()-odometer > self.SCAN_RATE and not self.is_arrived(precision=1.5):
                odometer = self.get_odometer()
                self.scan()
            self.schedule_movement()
            self.set_mode()
        self.path_planner.valid_target = False
        # self.__robot.movement.rotate(-self.path_planner.target_angle())  


    def is_arrived(self, precision=1):
        """
        :returns: True if the robot is near the cluster, False otherwise

        """
        return self.path_planner.target_distance() < max(self.path_planner.distance_allowed,self.TARGET_DISTANCE*precision) \
                or mpltPath.Path(self.path_planner.perimeter_target).contains_points([self.__robot.odometry.history[0][0]])
                

    def exploration(self):
        self.scheduler.exploration_mode()
        odometer = self.get_odometer()
        valid_target = self.scan()
        while not valid_target:
            self.exploration_movement()
            if self.get_odometer()-odometer > self.SCAN_RATE/2:
                odometer = self.get_odometer()
                valid_target = self.scan()
        utils.loginfo("Target: ({:.2f},{:.2f}) - Distance allowed: {:.2f}".format(self.path_planner.target[0],self.path_planner.target[1], max(self.path_planner.distance_allowed,self.TARGET_DISTANCE)))        
    
    def get_odometer(self):
        return self.__robot.odometry.history[0][1]

    def print_targets(self,target):
        file = open("/home/andrea/change_test.txt","a")
        string="[{},{}]\n".format(str(self.__robot.odometry.get_position()),str(target))
        file.write(string)
        file.close()

    def scan(self):
        self.__robot.movement.scan()
        targets = self.__robot.vision.locate_targets()
        #people_target=self.people_density.find_clusters(targets)
        clusters_targets=self.people_density.observation_update(targets) 
        #self.print_targets(targets)
        valid_target = self.path_planner.set_target(clusters_targets)
        return valid_target

    def schedule_movement(self):
        mode=self.scheduler.get_mode()

        if mode == 'bug':
            self.bug_movement()
        elif mode == 'potential_field':
            self.potential_field_movement()

    def set_mode(self):
        mode=self.scheduler.get_mode()
        if mode == 'bug':
            distance = utils.distance((self.__robot.odometry.x,self.__robot.odometry.y),self.loop_point)
            if distance > self.ESCAPING_DISTANCE:
                self.scheduler.potential_field_mode() 
        elif mode == 'potential_field' and self.check_loop():
            self.__robot.movement.rotate(180)
            self.scheduler.bug_mode()  


    def check_loop(self):
        """
        :returns: True if the robot is in a loop, False otherwise

        """
        position_loop=False
        lookback_window_size=min(self.LOOKBACK_WINDOW_SIZE,len(self.__robot.odometry.history))
        point_list = [(x,y) for ((x,y),_distance) in self.__robot.odometry.history[0:lookback_window_size]]
        for i in range(len(point_list)):
            for j in range(i+2,len(point_list)):
                if utils.distance(point_list[i],point_list[j])<self.MOVEMENT_LOOP_PRECISION:
                    position_loop=True
                    self.loop_point=point_list[i]
                    utils.loginfo("MOVING LOOP DETECTED")
                    break
        rotation_loop = self.__robot.odometry.history[0][1] - self.__robot.odometry.history[lookback_window_size-1][1]  < self.ROTATION_LOOP_PRECISION
        if rotation_loop:
            self.loop_point= self.__robot.odometry.history[0][0]  
            utils.loginfo("ROTATION LOOP DETECTED")
        return rotation_loop or position_loop

    def bug_movement(self):
        distance,angle = self.path_planner.bug_next_step()
        self.__robot.movement.rotate(angle)
        self.__robot.movement.move_forward(distance)

    def exploration_movement(self):
        distance,angle = self.path_planner.exploration_next_step()
        self.__robot.movement.rotate(angle)
        self.__robot.movement.move_forward(distance)    

    def potential_field_movement(self):
        angle = self.path_planner.next_step_direction()
        self.__robot.movement.rotate(angle)
        distance=self.path_planner.movement_distance()
        self.__robot.movement.move_forward(distance)

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

    
        
  


class Scheduler:
    def __init__(self):
        self.bug=False
        self.potential_field=False
        self.exploration=True

    def bug_mode(self):
        utils.loginfo("BUG MODE")
        self.bug=True
        self.potential_field=False
        self.exploration=False

    def potential_field_mode(self):
        utils.loginfo("POTENTIAL FIELD MODE")
        self.bug=False
        self.potential_field=True
        self.exploration=False

    def exploration_mode(self):
        utils.loginfo("EXPLORATION MODE")
        self.bug=False
        self.potential_field=False
        self.exploration=True

    def get_mode(self):
        for mode, flag in vars(self).items():
            if flag:
                return mode
