#!/usr/bin/env python3

import numpy as np
from numpy.linalg.linalg import norm
from sklearn.cluster import KMeans

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
print(X)


kmeans = KMeans(n_clusters=3)
kmeans.fit(X)
print(kmeans.cluster_centers_)
print(kmeans.labels_)