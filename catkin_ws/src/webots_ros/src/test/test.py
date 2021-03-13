#!/usr/bin/env python
import os
import ast
import math
import people_density as pd
import odometry
import controller
import vision
import path_planning
import clustering as clst
import matplotlib.pyplot as plt
from statistics import mean

show_map=False
show_points=True

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
        self.path_planner = path_planning.Path_planner(self) 

def get_data_old():
    path_to_repo="/Users/marco/GitHub/Robotics-project"
    path=path_to_repo+"/catkin_ws/src/webots_ros/src/change_pkg/log/log_polar_observation_dark.txt"
    result=[]
    size_cluster_ground_truth=0
    counter_cluster_ground_truth=0
    execution_number=0
    with open(path, 'r') as f:
        for line in f:
            execution_number+=1
            seeds={}
            observations={}
            # GROUND_TRUTH | 1_RUN | 2_RUN | 3_RUN | 4_RUN
            run_list=line.split("|")
            ground_truth=run_list.pop(0)
            number_of_run=len(run_list)
            ground_truth=ast.literal_eval(ground_truth)
            clusters_ground_truth,list_of_dimension = clst.clustering(ground_truth, 
                    distance_measure=clst.math_distance,
                    min_samples=2,
                    eps=2, 
                    dimensions=True)
            size_cluster_ground_truth+=sum(list_of_dimension)
            counter_cluster_ground_truth+=len(list_of_dimension)        
            for i,r in enumerate(run_list):
                # i_RUN = SEEDS # OBSERVATIONS # CLUSTER # ODOMETRY
                seed,observation,cluster,odometry = r.split("#")
                seeds[i+1]=ast.literal_eval(seed)
                observations[i+1]=(ast.literal_eval(odometry),ast.literal_eval(observation))
                #clusters[i+1]=ast.literal_eval(cluster)
            result.append({'ground_truth':ground_truth,'cluster_ground_truth':clusters_ground_truth,'observations':observations})
        average_cluster_size=size_cluster_ground_truth/counter_cluster_ground_truth
        average_cluster_number=counter_cluster_ground_truth/execution_number
    return result,average_cluster_size,average_cluster_number,number_of_run

def get_data():
    path_to_repo="/Users/marco/GitHub/Robotics-project"
    path=path_to_repo+"/catkin_ws/src/webots_ros/src/change_pkg/observations/output.txt"
    result=[]
    size_cluster_ground_truth=0
    counter_cluster_ground_truth=0
    average_cluster_size=0
    average_cluster_number=0
    execution_number=0
    with open(path, 'r') as f:
        for line in f:
            line=line[:-1]
            execution_number+=1
            observations={}
            # GROUND_TRUTH | 1_RUN | 2_RUN | 3_RUN | ... | i_RUN
            run_list=line.split("|")
            ground_truth=run_list.pop(0)
            number_of_run=len(run_list)
            ground_truth=ast.literal_eval(ground_truth)
            ground_truth=[(gt[1],gt[0]) for gt in ground_truth]
            clusters_ground_truth,list_of_dimension = clst.clustering(ground_truth, 
                    distance_measure=clst.math_distance,
                    min_samples=2,
                    eps=2, 
                    dimensions=True)
            size_cluster_ground_truth+=sum(list_of_dimension)
            counter_cluster_ground_truth+=len(list_of_dimension)        
            for i,r in enumerate(run_list):
                # i_RUN = 1_SCAN @ 2_SCAN @ 3_SCAN @ ... @ 7_SCAN
                scan_list=r.split("@")
                ids=set()
                for j,s in enumerate(scan_list):
                    # i_SCAN = OBSERVATIONS # ODOMETRY
                    data_pedestrian,odometry = s.split("#")
                    data_pedestrian=ast.literal_eval(data_pedestrian)
                    odometry=ast.literal_eval(odometry)
                    observation=[i[1] for i in data_pedestrian if i[0] not in ids]
                    for k in data_pedestrian:
                        ids.add(k[0]) 
                    if j==0:
                        odom=odometry
                        obs=observation
                    else:
                        obs+=change_ref(observation,odometry,odom)
                observations[i+1]=(odom,obs)
            result.append({'ground_truth':ground_truth,'cluster_ground_truth':clusters_ground_truth,'observations':observations})
        if counter_cluster_ground_truth>0:    
            average_cluster_size=size_cluster_ground_truth/counter_cluster_ground_truth
        if execution_number>0:    
            average_cluster_number=counter_cluster_ground_truth/execution_number
    return result,average_cluster_size,average_cluster_number,number_of_run

def change_ref(point_list,point_initial,point_final):
    return [ (p[0],-p[1]+point_initial[2]%360) for p in point_list] 

def print_clusters(clusters):
    for cluster in clusters:
        print("Center: ({:.2f},{:.2f})\nArea: {:.2f}".format(cluster['center'][0],cluster['center'][1],cluster['area']))

def print_point_list(clusters):
    for cluster in clusters:
        print("({:.2f},{:.2f})".format(cluster[0],cluster[1]),end=' ',flush=True)        

def main():
    data,average_cluster_size,average_cluster_number,number_of_run=get_data()
    true_positive={i+1:0 for i in range(number_of_run)}
    false_positive={i+1:0 for i in range(number_of_run)}
    false_negative={i+1:0 for i in range(number_of_run)}
    true_negative={i+1:0 for i in range(number_of_run)}
    false_negative_yolo={i+1:0 for i in range(number_of_run)}
    counter_observation={i+1:0 for i in range(number_of_run)}
    counter_ground_truth=0
    error_distance={i+1:[] for i in range(number_of_run)}
    #positions=[(2.5,-2.5),(2.5,2.5),(-2.5,2.5),(-2.5,-2.5)]
    for execution in data:
        seeds={}
        clusters={}
        observations_cartesian={}
        clusters_ground_truth=execution['cluster_ground_truth']
        ground_truth=execution['ground_truth']
        counter_ground_truth+=len(ground_truth)
        robot = Change()
        people_density = pd.GridMap(robot)
        for run,(odometry,observation) in execution['observations'].items():
            counter_observation[run]+=len(observation)
            robot.odometry.update(odometry)
            #observation=[robot.odometry.abs_cartesian_to_polar(i) for i in observation]
            #observation=[in_room(robot.odometry.polar_to_abs_cartesian(i)) for i in observation]
            
            observation=[i for i in observation if in_room(polar_to_abs_cartesian(i,odometry))]
            observations_cartesian[run]=[polar_to_abs_cartesian(i,odometry) for i in observation]
            cluster_dict,clusters_targets,s=people_density.observation_update(observation)
            seeds[run]=s
            error_distance[run]+=[distance(j,ground_truth)for j in seeds[run]]
            clusters[run]=clst.clustering(seeds[run], 
                    distance_measure=clst.math_distance,
                    min_samples=2,
                    eps=2) 

            for p in clusters_ground_truth:
                if not has_near(p,clusters[run]):
                    false_negative[run]+=1
                    if has_near(p,seeds[run]):
                        false_negative_yolo[run]+=1
            for p in clusters[run]:
                if has_near(p,clusters_ground_truth):
                    true_positive[run]+=1
                else:
                    false_positive[run]+=1
            for p in seeds[run]:
                if not has_near(p,clusters_ground_truth) and not has_near(p,clusters[run]):
                    true_negative[run]+=1 

            #robot.path_planner.set_target(cluster_dict)
            if show_map:
                people_density.draw_heat_map_inverted_centroids(clusters_targets) 
        if show_points:
            draw_clusters_all_run(execution['ground_truth'],observations_cartesian,clusters_ground_truth,seeds,clusters)
    
    report(false_positive,false_negative,false_negative_yolo,true_positive,true_negative,error_distance,counter_observation,counter_ground_truth,average_cluster_size,average_cluster_number)

def has_near(point,point_list,tollerance=1):
    for p in point_list:
        if clst.math_distance(p,point)<tollerance:
            return True
    return False

def only_in_room(point):
    x_inf=-5
    x_sup=5
    y_inf=-5
    y_sup=5
    return (min(max(x_inf,point[0]),x_sup),min(max(y_inf,point[1]),y_sup))

def in_room(point):
    x_inf=-5
    x_sup=5
    y_inf=-5
    y_sup=5
    return x_inf<=point[0]<=x_sup and y_inf<=point[1]<=y_sup

def report(false_positive,false_negative,false_negative_yolo,true_positive,true_negative,error_distance,counter_observation,counter_ground_truth,average_cluster_size,average_cluster_number):
    print("False positive: \t{}".format({k:round(v,2) for k,v in false_positive.items()}))        
    print("False negative: \t{}".format({k:round(v,2) for k,v in false_negative.items()}))
    print("False negative YOLO: \t{}".format({k:round(v,2) for k,v in false_negative_yolo.items()}))  
    print("True positive: \t\t{}".format({k:round(v,2) for k,v in true_positive.items()}))
    print("True negative: \t\t{}".format({k:round(v,2) for k,v in true_negative.items()}))
    accuracy={i:round((true_positive[i]+true_negative[i])/(true_positive[i]+false_positive[i]+true_negative[i]+false_negative[i]),2) for i in range(1,5)}
    precision={i:round(true_positive[i]/(true_positive[i]+false_positive[i]),2) for i in range(1,5)}
    recall={i:round(true_positive[i]/(true_positive[i]+false_negative[i]),2) for i in range(1,5)}
    recall_yolo={i:round(true_positive[i]/(true_positive[i]+false_negative_yolo[i]),2) for i in range(1,5)}
    print()
    print("Precision: \t{} \t- Mean: {:.2f}".format(precision,mean([i for i in precision.values()])))
    print("Accuracy: \t{} \t- Mean: {:.2f}".format(accuracy,mean([i for i in accuracy.values()])))
    print("Recall: \t{} \t- Mean: {:.2f}".format(recall,mean([i for i in recall.values()])))
    print("Recall YOLO: \t{} \t- Mean: {:.2f}".format(recall_yolo,mean([i for i in recall_yolo.values()])))
    print()
    print("Distance error: \t{}".format({k:round(mean([i for i in v if i<2]),2) for k,v in error_distance.items()}))
    print("Observation percentage: {}".format({k:round(v/counter_ground_truth,2) for k,v in counter_observation.items()}))
    print("Avarage cluster size: {:.2f}".format(average_cluster_size))
    print("Avarage cluster number: {:.2f}".format(average_cluster_number))


def draw_clusters_all_run(ground_truth,observations,centroid_ground_truth,seeds,centroids,title="Analysis"):
    x_g=[i[0] for i in ground_truth]
    y_g=[i[1] for i in ground_truth]
    x_c_g=[i[0] for i in centroid_ground_truth]
    y_c_g=[i[1] for i in centroid_ground_truth]
    a=3
    b=3
    fig, ax = plt.subplots(a, b, figsize=(9, 9))
    fig.suptitle(title)
    positions=[[],[2,0],[1,0],[0,0],[0,1],[0,2],[1,2],[2,2],[2,1]]
    for i, values in seeds.items():
        x_c=[k[0] for k in centroids[i]]
        y_c=[k[1] for k in centroids[i]]
        x_o=[k[0] for k in observations[i]]
        y_o=[k[1] for k in observations[i]]
        x_s=[i[0] for i in values]
        y_s=[i[1] for i in values]
        kx,ky=positions[i]
        ax[kx,ky].axis("equal")
        ax[kx,ky].set_xlim(-5,5)
        ax[kx,ky].set_ylim(-5,5)
        ax[kx,ky].set_xlabel('Position (m)')
        ax[kx,ky].set_ylabel('Position (m)')
        ax[kx,ky].scatter(x_c_g,y_c_g, color="y", s=100)
        ax[kx,ky].scatter(x_c,y_c, color="r", s=70)
        ax[kx,ky].scatter(x_s,y_s, color="g", s=50)
        ax[kx,ky].scatter(x_g,y_g, color="b", s=30)
        ax[kx,ky].scatter(x_o,y_o, color="k", s=20)
        #ax[kx,ky].invert_xaxis()
    ax[1,1].legend(('Ground Truth centroids', 'Centroids', 'Seeds','Ground Truth','Observations'),bbox_to_anchor=(0.5, 0.5))    
    plt.show()    

def distance(point,point_list):
    return min([clst.math_distance(point,i) for i in point_list])


def abs_cartesian_to_polar(p, odom):
    """Accepts a point given in cartesian coordinates relative to the world
    frame of reference and converts it to polar coordinates in the robot
    frame of reference based on the current position estimate

    :p:   Cartesian coordinates in the (x, y) format
    :returns:  Cartesian coordinates in the (Rho, Theta) format

    """
    x = p[0] - odom[0]
    y = p[1] - odom[1]
    angle = -180/math.pi*math.atan2(x,y)-odom[2]
    angle = angle if angle < 180 else angle -360
    return (math.hypot(x,y), angle)


def polar_to_abs_cartesian(p, odom):
    """Accepts a point given in polar coordinates relative to the robot
    frame of reference and converts it to cartesian coordinates in a wolrd
    frame of reference based on the current position estimate

    :p:   Polar coordinates in the (Rho, Theta) format
    :returns:       Cartesian coordinates

    """
    angle=p[1]-odom[2]
    return (odom[1]+p[0]*math.sin(math.pi*angle/180),
            odom[0]+p[0]*math.cos(math.pi*angle/180))


if __name__ == '__main__':
   main()  
