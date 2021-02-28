import ast

with open("./log.txt", 'r') as f:
    for line in f:
        # GROUND_TRUTH | 1_RUN | 2_RUN | 3_RUN | 4_RUN
        [ground_truth,run_1,run_2,run_3,run_4]=line.split("|")
        ground_truth=ast.literal_eval(ground_truth)
        for r in [run_1,run_2,run_3,run_4]:
            # i_RUN = SEEDS - OBSERVATIONS - CLUSTER 
            seeds,observations,cluster = r.split("-")
            seeds=ast.literal_eval(seeds)
            observations=ast.literal_eval(observations)
            cluster=ast.literal_eval(cluster)
