#!/usr/bin/env python3

import numpy as np
from numpy.linalg.linalg import norm

fc = np.ones((3,10))

fc[2,1:9] = fc[2, 1:9] + 9.0

vec = np.ones((3,1))
vec[2] = 3


norm_vec = np.array([[1],[2],[3]])
print(np.transpose(norm_vec))
print(norm_vec)