#!/usr/bin/env python
import os
import ast
import people_density as pd
import odometry
import controller
import vision
import path_planning
import clustering as clst
import matplotlib.pyplot as plt

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

def get_data():
    path_to_repo="/Users/marco/GitHub/Robotics-project"
    path=path_to_repo+"/catkin_ws/src/webots_ros/src/change_pkg/log/log_polar_observation.txt"
    result=[]
    with open(path, 'r') as f:
        for line in f:
            seeds={}
            observations={}
            # GROUND_TRUTH | 1_RUN | 2_RUN | 3_RUN | 4_RUN
            [ground_truth,run_1,run_2,run_3,run_4]=line.split("|")
            ground_truth=ast.literal_eval(ground_truth)
            clusters_ground_truth = clst.clustering(ground_truth, 
                    distance_measure=clst.math_distance,
                    min_samples=2,
                    eps=2)
            for i,r in enumerate([run_1,run_2,run_3,run_4]):
                # i_RUN = SEEDS # OBSERVATIONS # CLUSTER 
                seed,observation,cluster = r.split("#")
                seeds[i+1]=ast.literal_eval(seed)
                observations[i+1]=ast.literal_eval(observation)
                #clusters[i+1]=ast.literal_eval(cluster)
            result.append({'ground_truth':ground_truth,'cluster_ground_truth':clusters_ground_truth,'observations':observations})

    return result
 

def print_clusters(clusters):
    for cluster in clusters:
        print("Center: ({:.2f},{:.2f})\nArea: {:.2f}".format(cluster['center'][0],cluster['center'][1],cluster['area']))

def print_point_list(clusters):
    for cluster in clusters:
        print("({:.2f},{:.2f})".format(cluster[0],cluster[1]),end=' ',flush=True)        

def main():
    true_positive={1:0,2:0,3:0,4:0}
    false_positive={1:0,2:0,3:0,4:0}
    false_negative={1:0,2:0,3:0,4:0}
    true_negative={1:0,2:0,3:0,4:0}
    false_negative_yolo={1:0,2:0,3:0,4:0}
    data=get_data()
    positions=[(2.5,-2.5),(2.5,2.5),(-2.5,2.5),(-2.5,-2.5)]
    for execution in data:
        seeds={}
        clusters={}
        clusters_ground_truth=execution['cluster_ground_truth']
        robot = Change()
        people_density = pd.GridMap(robot)
        for run,observation in execution['observations'].items():
            robot.odometry.update((positions[run-1][0],positions[run-1][1],0))
            #observation=[robot.odometry.abs_cartesian_to_polar(i) for i in observation]
            cluster_dict,clusters_targets,s=people_density.observation_update(observation)
            seeds[run]=s
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
            #people_density.draw_heat_map_inverted_centroids(clusters_targets)
        #draw_clusters_all_run(execution['ground_truth'],clusters_ground_truth,seeds,clusters)
    report(false_positive,false_negative,false_negative_yolo,true_positive,true_negative)

def has_near(point,point_list,tollerance=1):
    for p in point_list:
        if clst.math_distance(p,point)<tollerance:
            return True
    return False


def report(false_positive,false_negative,false_negative_yolo,true_positive,true_negative):
    print("False positive: {}".format({k:round(v,2) for k,v in false_positive.items()}))        
    print("False negative: {}".format({k:round(v,2) for k,v in false_negative.items()}))
    print("False negative for YOLO: {}".format({k:round(v,2) for k,v in false_negative_yolo.items()}))  
    print("True positive: {}".format({k:round(v,2) for k,v in true_positive.items()}))
    print("True negative: {}".format({k:round(v,2) for k,v in true_negative.items()}))

    accuracy={i:round((true_positive[i]+true_negative[i])/(true_positive[i]+false_positive[i]+true_negative[i]+false_negative[i]),2) for i in range(1,5)}
    precision={i:round(true_positive[i]/(true_positive[i]+false_positive[i]),2) for i in range(1,5)}
    recall={i:round(true_positive[i]/(true_positive[i]+false_negative[i]),2) for i in range(1,5)}
    recall_yolo={i:round(true_positive[i]/(true_positive[i]+false_negative[i]-false_negative_yolo[i]),2) for i in range(1,5)}
    print("Precision: {}".format(precision))
    print("Accuracy: {}".format(accuracy))
    print("Recall: {}".format(recall))
    print("Recall YOLO: {}".format(recall_yolo))

def draw_clusters_all_run(observations,centroid_ground_truth,seeds,centroids,title="Analysis"):
    x_o=[i[0] for i in observations]
    y_o=[i[1] for i in observations]
    x_c_g=[i[0] for i in centroid_ground_truth]
    y_c_g=[i[1] for i in centroid_ground_truth]
    fig, ax = plt.subplots(2, 2, figsize=(8, 8))
    fig.suptitle(title)
    positions=[[],[0,0],[0,1],[1,0],[1,1]]
    for i, values in seeds.items():
        x_c=[k[0] for k in centroids[i]]
        y_c=[k[1] for k in centroids[i]]
        x_s=[i[0] for i in values]
        y_s=[i[1] for i in values]
        kx,ky=positions[i]
        ax[kx,ky].axis("equal")
        ax[kx,ky].set_xlabel('Position (m)')
        ax[kx,ky].set_ylabel('Position (m)')
        ax[kx,ky].scatter(x_c_g,y_c_g, color="y", s=100)
        ax[kx,ky].scatter(x_c,y_c, color="r", s=70)
        ax[kx,ky].scatter(x_s,y_s, color="g", s=50)
        ax[kx,ky].scatter(x_o,y_o, color="b", s=30)
        
        ax[kx,ky].invert_xaxis()
    ax[1,1].legend(('Ground Truth centroids', 'Centroids', 'Seeds','Ground Truth'),bbox_to_anchor=(0.5, 0.5))    
    plt.show()    


if __name__ == '__main__':
   main()  
