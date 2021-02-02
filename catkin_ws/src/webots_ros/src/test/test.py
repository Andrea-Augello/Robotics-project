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
    f = open(os.path.dirname(__file__)+"/test_positions/change_test.txt", "r")
    observations=[]
    for x in f:
        observations.append(ast.literal_eval(x))
    f.close()
    return observations
 

def print_clusters(clusters):
    for cluster in clusters:
        print("Center: ({:.2f},{:.2f})\nArea: {:.2f}".format(cluster['center'][0],cluster['center'][1],cluster['area']))

def print_point_list(clusters):
    for cluster in clusters:
        print("({:.2f},{:.2f})".format(cluster[0],cluster[1]),end=' ',flush=True)        

def main():
    robot = Change()
    people_density = pd.GridMap(robot)
    data=get_data()
    for observation in data:
        robot.odometry.update(observation[0])
        clusters_targets=people_density.observation_update(observation[1])
        #print(robot.odometry.get_position())
        #print_point_list(observation[1])
        #print()
        print_point_list(clusters_targets)
        print()
        #print_clusters(clusters_targets)
        people_density.draw_heat_map_inverted_centroids(clusters_targets)

if __name__ == '__main__':
   main()  


  

