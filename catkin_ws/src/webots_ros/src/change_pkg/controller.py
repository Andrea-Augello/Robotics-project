#!/usr/bin/env python
class Controller:
    def __init__(self, robot):
        self.__robot=robot
        self.path_planner = path_planner.Path_planner(robot)
