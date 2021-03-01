import numpy as np
import math
from sklearn.cluster import DBSCAN
from sklearn import metrics
from sklearn.datasets import make_blobs
from sklearn.preprocessing import StandardScaler


def arc_distance(p1,p2):
    angle_diff = (p1[1]-p2[1])%360
    angle_diff = angle_diff if angle_diff < 180 else abs(angle_diff-360)
    angle_diff = angle_diff if angle_diff < 57 else 999999
    angle_diff = angle_diff if angle_diff > 1 else 0
    lin_diff = math.sqrt(abs(p1[0]**2+p2[0]**2-2*p1[0]*p2[0]*math.cos((p1[1]-p2[1])*math.pi/180)))
    #print(angle_diff*lin_diff)
    return angle_diff*lin_diff #min(angle_diff, lin_diff)

def math_distance(p1,p2):
    return math.hypot(p1[0]-p2[0],p1[1]-p2[1])

def clustering(points, distance_measure=arc_distance, min_samples=1, eps=0.5):
        if not len(points):
            return []
        scaler = StandardScaler().fit(points)
        X = scaler.inverse_transform(scaler.transform(points))
        # Compute DBSCAN
        db = DBSCAN(eps=eps, min_samples=min_samples, metric=distance_measure).fit(X)
        core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
        core_samples_mask[db.core_sample_indices_] = True
        labels = db.labels_

        # Number of clusters in labels, ignoring noise if present.
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
         
        clustered_centers_angle_median=[]
        clustered_centers_radius_median=[]
        for k in range(n_clusters_):
                my_members = labels == k
                clustered_centers_angle_median.append(np.median(X[my_members,1]))
                clustered_centers_radius_median.append(np.median(X[my_members,0]))
        output=[]
        counter=0
        for point in clustered_centers_radius_median:
                newpoint=[]
                newpoint.append(point)
                newpoint.append(clustered_centers_angle_median[counter])
                output.append(newpoint)
                counter+=1      
        return output
