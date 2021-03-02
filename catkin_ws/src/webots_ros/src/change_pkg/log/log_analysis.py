import ast
import clustering as clst
import matplotlib.pyplot as plt
import math

def main():
    path_to_repo="/Users/marco/GitHub/Robotics-project"
    path=path_to_repo+"/catkin_ws/src/webots_ros/src/change_pkg/log/log_polar_observation.txt"
    true_positive={1:0,2:0,3:0,4:0}
    false_positive={1:0,2:0,3:0,4:0}
    false_negative={1:0,2:0,3:0,4:0}
    true_negative={1:0,2:0,3:0,4:0}
    false_negative_yolo={1:0,2:0,3:0,4:0}

    true_positive_old={1:0,2:0,3:0,4:0}
    false_positive_old={1:0,2:0,3:0,4:0}
    false_negative_old={1:0,2:0,3:0,4:0}
    true_negative_old={1:0,2:0,3:0,4:0}
    false_negative_yolo_old={1:0,2:0,3:0,4:0}
    positions=[(2.5,-2.5),(2.5,2.5),(-2.5,2.5),(-2.5,-2.5)]

    with open(path, 'r') as f:
        for line in f:
            seeds={}
            observations={}
            clusters={}
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
                observations[i+1]=[polar_to_abs_cartesian(j,positions[i],0) for j in observations[i+1]]
                #clusters[i+1]=ast.literal_eval(cluster)
                clusters[i+1]=clst.clustering(seeds[i+1], 
                    distance_measure=clst.math_distance,
                    min_samples=2,
                    eps=2) 
                for p in clusters_ground_truth:
                    if not has_near(p,clusters[i+1]):
                        false_negative[i+1]+=1
                        if has_near(p,seeds[i+1]):
                            false_negative_yolo[i+1]+=1
                for p in clusters[i+1]:
                    if has_near(p,clusters_ground_truth):
                        true_positive[i+1]+=1
                    else:
                        false_positive[i+1]+=1
                for p in seeds[i+1]:
                    if not has_near(p,clusters_ground_truth) and not has_near(p,clusters[i+1]):
                        true_negative[i+1]+=1 


                #OLD METHOD
                observations[i+1]=clst.clustering(observations[i+1], 
                    distance_measure=clst.math_distance,
                    min_samples=2,
                    eps=2) 
                for p in clusters_ground_truth:
                    if not has_near(p,observations[i+1]):
                        false_negative_old[i+1]+=1
                        if has_near(p,seeds[i+1]):
                            false_negative_yolo_old[i+1]+=1
                for p in observations[i+1]:
                    if has_near(p,clusters_ground_truth):
                        true_positive_old[i+1]+=1
                    else:
                        false_positive_old[i+1]+=1
                for p in seeds[i+1]:
                    if not has_near(p,clusters_ground_truth) and not has_near(p,observations[i+1]):
                        true_negative_old[i+1]+=1                            

            
            #draw_clusters_all_run(ground_truth,clusters_ground_truth,seeds,clusters)
    print("NEW METHOD\n")
    report(false_positive,false_negative,false_negative_yolo,true_positive,true_negative)
    print("\nOLD METHOD\n")
    report(false_positive_old,false_negative_old,false_negative_yolo_old,true_positive_old,true_negative_old)

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

def draw_clusters(observations,centroids,title):
        x_o=[i[0] for i in observations]
        y_o=[i[1] for i in observations]
        x_c=[i[0] for i in centroids]
        y_c=[i[1] for i in centroids]

        fig, ax1 = plt.subplots(1, 1, figsize=(5, 5))
        fig.suptitle(title)
        ax1.axis("equal")
        ax1.set_xlabel('Position (m)')
        ax1.set_ylabel('Position (m)')
        ax1.scatter(x_c,y_c, color="y", s=100)
        ax1.scatter(x_o,y_o, color="b")
        ax1.invert_xaxis()
        plt.show()

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

def polar_to_abs_cartesian(p, coord,theta):
        """Accepts a point given in polar coordinates relative to the robot
        frame of reference and converts it to cartesian coordinates in a wolrd
        frame of reference based on the current position estimate

        :p:   Polar coordinates in the (Rho, Theta) format
        :returns:       Cartesian coordinates

        """
        x,y=coord
        angle=p[1]-theta
        return (x+p[0]*math.sin(math.pi*angle/180),
                y+p[0]*math.cos(math.pi*angle/180))

def coord_to_point(coord):
    xy_resolution=0.2
    min_x=-10
    min_y=-10
    x=coord[0]*xy_resolution+min_x
    y=coord[1]*xy_resolution+min_y
    return (x,y)        

if __name__ == "__main__":
    main()                      