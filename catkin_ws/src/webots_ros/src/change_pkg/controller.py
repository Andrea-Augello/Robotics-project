#!/usr/bin/env python

import change_pkg.path_planning as path_planner
import change_pkg.utils as utils
import itertools
import numpy as np
import change_pkg.people_density as people_density
import matplotlib.pyplot as plt
import matplotlib.path as mpltPath
import time

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
        self.TARGET_DISTANCE = 1
        self.SCAN_RATE = 10

    def start(self):
        while True:
            pass
            self.__robot.movement.move_forward(3)
            self.__robot.movement.rotate(-90)
            self.__robot.odometry.movement_history()
        while False:
            # Sensor characterization
            self.__robot.vision.clear_saved_frames()
            self.__robot.vision.save_frame(self.__robot.sensors.camera.value)
            utils.loginfo(self.__robot.vision.locate_targets())
            time.sleep(3)
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
            if self.get_odometer()-odometer > self.SCAN_RATE:
                odometer = self.get_odometer()
                self.scan()
            self.schedule_movement()
            self.set_mode()
        self.path_planner.valid_target = False
        # self.__robot.movement.rotate(-self.path_planner.target_angle())  


    def is_arrived(self):
        """
        :returns: True if the robot is near the cluster, False otherwise

        """
        return self.path_planner.target_distance() < max(self.path_planner.distance_allowed,self.TARGET_DISTANCE) \
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

    def scan(self):
        self.__robot.movement.scan()
        targets = self.__robot.vision.locate_targets()
        #people_target=self.people_density.find_clusters(targets)
        clusters_targets=self.people_density.observation_update(targets) 
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
