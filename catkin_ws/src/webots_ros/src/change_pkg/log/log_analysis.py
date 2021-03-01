import ast
import clustering as clst
import matplotlib.pyplot as plt

def main():
    path_to_repo="/Users/marco/GitHub/Robotics-project"
    path=path_to_repo+"/catkin_ws/src/webots_ros/src/change_pkg/log/log.txt"

    with open(path, 'r') as f:
        for line in f:
            seeds={}
            observations={}
            clusters={}
            # GROUND_TRUTH | 1_RUN | 2_RUN | 3_RUN | 4_RUN
            [ground_truth,run_1,run_2,run_3,run_4]=line.split("|")
            ground_truth=ast.literal_eval(ground_truth)
            for i,r in enumerate([run_1,run_2,run_3,run_4]):
                # i_RUN = SEEDS # OBSERVATIONS # CLUSTER 
                seed,observation,cluster = r.split("#")
                seeds[i+1]=ast.literal_eval(seed)
                observations[i+1]=ast.literal_eval(observation)
                clusters[i+1]=ast.literal_eval(cluster)

            clusters_ground_truth = clst.clustering(ground_truth, 
                    distance_measure=clst.math_distance,
                    min_samples=2,
                    eps=1.5)

            draw_clusters(ground_truth,clusters_ground_truth)

def draw_clusters(observations,centroids):
        x_o=[i[0] for i in observations]
        y_o=[i[1] for i in observations]
        x_c=[i[0] for i in centroids]
        y_c=[i[1] for i in centroids]

        fig, ax1 = plt.subplots(1, 1, figsize=(5, 5))
        fig.suptitle('People recognition')
        ax1.axis("equal")
        ax1.set_xlabel('Position (m)')
        ax1.set_ylabel('Position (m)')
        ax1.scatter(x_c,y_c, color="y", s=100)
        ax1.scatter(x_o,y_o, color="b")
        ax1.invert_xaxis()
        plt.show()

def coord_to_point(coord):
    xy_resolution=0.2
    min_x=-10
    min_y=-10
    x=coord[0]*xy_resolution+min_x
    y=coord[1]*xy_resolution+min_y
    return (x,y)        

if __name__ == "__main__":
    main()                      