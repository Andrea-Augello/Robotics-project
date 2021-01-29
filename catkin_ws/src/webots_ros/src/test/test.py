#!/usr/bin/env python
import os
import ast
import people_density as pd
import odometry
import controller
import vision

class Change:
    def __init__(self):
        self.name = 'change'
        self.debug_mode = False
        self.time_step = 16
        self.wheel_diameter = 0.20
        self.footprint = 0.54
        self.vision = vision.Vision()
        self.odometry = odometry.Odometry()
        self.controller = controller.Controller(self) 

def get_data():
    f = open(os.path.dirname(__file__)+"/test_positions/test.txt", "r")
    observations=[]
    for x in f:
        observations.append(ast.literal_eval(x))
    f.close()
    return observations

def trasform(targets):
    return [(-i[1],i[0]) for i in targets]   

def print_clusters(clusters):
    for cluster in clusters:
        print("Center: ({:.2f},{:.2f})\nArea: {:.2f}".format(cluster['center'][0],cluster['center'][1],cluster['area']))

def main():
    robot = Change()
    people_density = pd.GridMap(robot)
    data=get_data()
    clusters_targets=people_density.observation_update(trasform(data[0][1]))
    print(trasform(data[0][1]))
    print_clusters(clusters_targets)
    people_density.draw_heat_map()

if __name__ == '__main__':
   main()  


  

