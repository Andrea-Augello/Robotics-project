#!/usr/bin/env python

import change_pkg.path_planning as path_planner

class Controller:
    def __init__(self, robot):
        self.__robot=robot
        self.path_planner = path_planner.Path_planner(robot)

    def start(self):
        valid_target = False
        bugmode = 0
        distances = [0,0,0,0]
        while not valid_target:
            self.__robot.movement.scan()
            valid_target = self.path_planner.set_target(self.path_planner.find_clusters(self.__robot.vision.locate_targets()))
        while self.path_planner.target_distance() > 0.5:
            self.__robot.print_info()
            if bugmode > 1:
                distance,angle = self.path_planner.bug_next_step()
                self.__robot.movement.rotate(angle)
                self.__robot.movement.move_forward(distance)
                bugmode = bugmode - 1
            else :
                angle = self.path_planner.next_step_direction()
                self.__robot.movement.rotate(angle)
                distance=self.path_planner.movement_distance()
                self.__robot.movement.move_forward(distance)
                distances.pop(0)
                distances.append(self.__robot.odometry.distance_traveled)
                if distances[3] - distances[0] < 0.1:
                    bugmode = 3
        self.__robot.movement.rotate(-self.path_planner.target_angle())
        self.__robot.print_info()    
        self.__robot.odometry.movement_history()