import numpy as np

import math
from sklearn.cluster import DBSCAN
from sklearn import metrics
from sklearn.datasets import make_blobs
from sklearn.preprocessing import StandardScaler
import matplotlib.pyplot as plt
from itertools import cycle


# #############################################################################
# Generate sample data
centers = [[1.6326544740621143, 280.2121740625],
[6.584455987498526, 304.47335590625005],
[6.929602039689195, 312.39628798437496],
[6.200314562627878, 304.65240521875],
[5.804895618419142, 312.4410503125],
[6.675169273969831, 313.560108515625],
[6.539154460747976, 304.518118234375],
[6.658405156507997, 312.4410503125],
[6.02878028159751, 304.65240521875],
[5.5969133687342785, 312.4410503125],
[5.940518754916619, 313.42582153125005],
[1.285950569200028, 361.72437357812504],
[1.399689127232946, 361.76913590625]]

plt.polar([ c[1] for c in centers], [ c[0] for c in centers], 'o')
scaler = StandardScaler().fit(centers)

X, labels_true = make_blobs(n_samples=len(centers), centers=centers, cluster_std=0.4, random_state=0)

X = scaler.inverse_transform(scaler.transform(centers))

# #############################################################################
def distance(p1,p2):
    angle_diff = abs(p1[1]-p2[1])%360
    angle_diff = angle_diff if angle_diff < 57 else 999999
    angle_diff = angle_diff if angle_diff > 1 else 0
    lin_diff = math.sqrt(p1[0]**2+p2[0]**2-2*p1[0]*p2[0]*math.cos((p1[1]-p2[1])*math.pi/180))
    print(angle_diff*lin_diff)
    return angle_diff*lin_diff #min(angle_diff, lin_diff)

# Compute DBSCAN
db = DBSCAN(eps=0.5, min_samples=1, metric=distance ).fit(X)
core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
core_samples_mask[db.core_sample_indices_] = True
labels = db.labels_

# Number of clusters in labels, ignoring noise if present.
n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
n_noise_ = list(labels).count(-1)

print('Estimated number of clusters: %d' % n_clusters_)
print('Estimated number of noise points: %d' % n_noise_)
print("Homogeneity: %0.3f" % metrics.homogeneity_score(labels_true, labels))
print("Completeness: %0.3f" % metrics.completeness_score(labels_true, labels))
print("V-measure: %0.3f" % metrics.v_measure_score(labels_true, labels))
print("Adjusted Rand Index: %0.3f"
      % metrics.adjusted_rand_score(labels_true, labels))
print("Adjusted Mutual Information: %0.3f"
      % metrics.adjusted_mutual_info_score(labels_true, labels))
print("Silhouette Coefficient: %0.3f"
      % metrics.silhouette_score(X, labels))

# #############################################################################
# Plot result

plt.figure(1)
plt.clf()

#X = scaler.inverse_transform(X)
colors = cycle('bgrcmykbgrcmykbgrcmykbgrcmyk')
for k, col in zip(range(n_clusters_), colors):
    my_members = labels == k
    #cluster_center = cluster_centers[k]
    plt.plot(X[my_members, 1], X[my_members, 0], col + '.')
    #plt.polar(cluster_center[1], cluster_center[0], 'o', markerfacecolor=col, markeredgecolor='k', markersize=14)
plt.title('Estimated number of clusters: %d' % n_clusters_)
plt.show()
