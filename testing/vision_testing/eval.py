#!/usr/bin/env python3

import statistics
import numpy as np
from wire_modeling.wire_grasp_toolbox import WireGraspToolbox
import math
import os

wire_thickness = 3
test_number = 3

save_path = '/usr/local/src/wire_manipulation_framework/src/wire_manipulation/testing/vision_testing/wire' + str(wire_thickness) + '/test' + str(test_number)
name_of_ground_truth_file = "vision_test_" + str(test_number) + "_ground_truth"  
name_of_sensed_file = "vision_test_" + str(test_number) + "_sensed_truth"  
name_of_statistics_file = "test_" + str(test_number) + "_statistics"

GroundTruth = os.path.join(save_path, name_of_ground_truth_file +".txt")
Sensed = os.path.join(save_path, name_of_sensed_file +".txt")
exp_statistics = os.path.join(save_path, name_of_statistics_file +".txt")

f_gt = open(GroundTruth, 'r+')
f_s = open(Sensed, 'r+')
f_stats = open(exp_statistics, 'w')

# get sensed data 
s = f_s.readlines()
sensed_data = np.zeros((len(s),3))
i = 0
for sensed in s:
    sensed = str(sensed)
    sensed_data[[i],[0]] = float(sensed.split(' ')[0])
    sensed_data[[i],[1]] = float(sensed.split(' ')[1])
    sensed_data[[i],[2]] = float(sensed.split(' ')[2])
    i = i+1

# get ground Truth
gt = f_gt.readlines()
ground_truth_data = np.zeros((len(gt),3))
i = 0
for ground_truth in gt:
    ground_truth = str(ground_truth)
    ground_truth_data[[i],[0]] = float(ground_truth.split(' ')[0])
    ground_truth_data[[i],[1]] = float(ground_truth.split(' ')[1])
    ground_truth_data[[i],[2]] = float(ground_truth.split(' ')[2])
    i = i+1


error3 = []
error2 = []


for i in range(len(gt)):
    min_3_dist = 5
    min_2_dist = 5
    for j in range(len(s)):
        three_D = ((ground_truth_data[i][0] - sensed_data[j][0])**2 + (ground_truth_data[i][1] - sensed_data[j][1])**2
                + (ground_truth_data[i][2] - sensed_data[j][2])**2)**0.5

        two_D = ( (ground_truth_data[i][1] - sensed_data[j][1])**2
                + (ground_truth_data[i][2] - sensed_data[j][2])**2)**0.5

        if three_D < min_3_dist:
            min_3_dist = three_D

        if two_D < min_2_dist:
            min_2_dist = two_D

    error3.append(min_3_dist)
    error2.append(min_2_dist)

print(error3)
print("")
print(error2)


# Standard deviation of list
# Using sum() + list comprehension
mean3 = sum(error3) / len(error3)
variance3 = sum([((x - mean3) ** 2) for x in error3]) / len(error3)
res3 = variance3 ** 0.5

mean2 = sum(error2) / len(error2)
variance2 = sum([((x - mean2) ** 2) for x in error2]) / len(error2)
res2 = variance2 ** 0.5

f_stats.writelines(["3D Statistics","\n"])
f_stats.writelines(["MEAN ERROR: ",str(mean3),"\n"])
f_stats.writelines(["Variance: ",str(variance3),"\n"])
f_stats.writelines(["STD: ",str(res3),"\n"])

f_stats.writelines(["2D Statistics","\n"])
f_stats.writelines(["MEAN ERROR: ",str(mean2),"\n"])
f_stats.writelines(["Variance: ",str(variance2),"\n"])
f_stats.writelines(["STD: ",str(res2),"\n"])


