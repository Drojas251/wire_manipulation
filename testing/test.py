#!/usr/bin/env python3

import numpy as np
from numpy.linalg.linalg import norm
from sklearn.cluster import KMeans
import bezier
from wire_modeling.Bezier import Bezier

fc = np.ones((3,10))

fc[2,1:9] = fc[2, 1:9] + 9.0

vec = np.ones((3,1))
vec[2] = 3

X = np.ones((3,10))
X[[2],[1]] = 6
X[[1],[6]] = 4
X[[2],[4]] = 12
X[[0],[8]] = 3
X[[1],[5]] = 2
X[[0],[1]] = 7

X = np.transpose(X)



#kmeans = KMeans(n_clusters=3)
#kmeans.fit(X)
#print(kmeans.cluster_centers_)
#print(kmeans.labels_)


points_set_1 = np.array([[0, 0, 0], [0, 4, 0], [2, 5, 0], [4, 5, 0], [5, 4, 0], [5, 1, 0], [4, 0, 0], [1, 0, 3], [0, 0, 4], [0, 2, 5], [0, 4, 5], [4, 5, 5], [5, 5, 4], [5, 5, 0]])
print(points_set_1)
t_points = np.arange(0, 1, 0.01)
curve_set_1 = Bezier.Curve(t_points, points_set_1)
print(curve_set_1.size)