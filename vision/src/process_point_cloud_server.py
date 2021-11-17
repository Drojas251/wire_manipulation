#!/usr/bin/env python3
import rospy
#from __future__ import print_function
from wire_modeling_msgs.srv import ProcessPointCloud, ProcessPointCloudResponse
import math

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

import ros_numpy
import numpy as np
from sklearn.cluster import KMeans
from wire_modeling.Bezier import Bezier
from wire_modeling.wire_sim import *

def sort_points(points,end_point):
    elements = []
    elements.append(end_point)

    sorted_points = np.zeros((len(points), 3))
    sorted_points[0,:] = points[int(end_point),:]

    for i in range(len(points) - 1):
        min_dist = 5
        for j in range(len(points)):
            compare = True
            for ele in elements:
                if j == ele:
                    compare = False
            if compare == True:
                dist = ((points[j,0] - sorted_points[i,0])**2 + (points[j,1] - sorted_points[i,1])**2 + (points[j,2] - sorted_points[i,2])**2 )**0.5
                if dist < min_dist:
                    min_dist = dist
                    element = j
        sorted_points[i+1,:] = points[element,:]
        elements.append(element)
    return sorted_points

def get_final_node_set(sorted_points,N):
    # N = number of nodes
    t_points = np.arange(0, 1, 0.01)
    curve_set = Bezier.Curve(t_points, sorted_points)

    final_node_set = np.zeros((N,3))
    step = 5

    for i in range(20):
        final_node_set[[i],:] = curve_set[[i*5],:]

    return final_node_set 

def process_point_cloud(req):
    pc = ros_numpy.numpify(req.pointcloud)
    points=np.zeros((len(pc)*len(pc[0]),3))
    count = 0
    N = 20

    translation = np.array([0, 0, 0])

    # format the processed pointcloud into a N x 3 numpy array
    for x in range(len(pc)):
        for y in range(len(pc[0])):
            points2 = pc[x,y]
            PP = [points2[0],points2[1],points2[2]]
            if not math.isnan(points2[1]):
                # transform the points into the world frame
                points[count] = [points2[0],points2[1],points2[2]] + translation  
                count = count + 1

    # After "count" all elements should be zero. Extract only non-zero elements
    points_ = points[0:(count-1), :]

    # cluster the point cloud data into N = 20 nodes
    kmeans = KMeans(n_clusters=N)
    kmeans.fit(points_)
    #print(kmeans.labels_)
    C = kmeans.cluster_centers_

    # Find outliers 
    outliers = []
    threshold = 0.05 # a point whos closest neighbor is greater than 0.1m away is considered an outlier 

    for k in range(N):
        min_dist = 5
        for j in range(N):
            if k != j:
                dist = ((C[k,0] - C[j,0])**2 + (C[k,1] - C[j,1])**2 + (C[k,2] - C[j,2])**2 )**0.5
                if dist < min_dist:
                    min_dist = dist
            
        # check outlier condition    
        if min_dist > threshold: 
            outliers.append(k)
        
    # Get rid of outliers 
    # reduce your array
    size_of_outliers = len(outliers)
    print('# of Outliers',size_of_outliers)
    new_points = np.zeros((N-size_of_outliers,3))
    j = 0

    for i in range(N):
        remove = False
        for k in range(size_of_outliers):
            if i == outliers[k]:
                remove = True
        if(remove != True):
            new_points[[j],:] = C[[i],:]
            j = j +1


    # check the extreme points ( maz min x,y,z)
    # find the element with the most extreme values
    # ex. element 6 might hold the max x and min z. Likley an end point
    # use this point as a starting point

    min_y = max_y = new_points[0,1]
    min_x = max_x = new_points[0,0]
    extrema_elements = np.zeros(4) # [min_y, max_y, min_z, max_z]

    for j in range(len(new_points)):
        if new_points[j,1] <= min_y:
            min_y = new_points[j,1]
            extrema_elements[0] = int(j)
        if new_points[j,1] >= max_y:
            max_y = new_points[j,1]
            extrema_elements[1] = int(j)
        if new_points[j,0] <= min_x:
            min_x = new_points[j,0]
            extrema_elements[2] = int(j)
        if new_points[j,0] >= max_x:
            max_x = new_points[j,0]
            extrema_elements[3] = int(j)

    #print('min_y', min_y)
    #print('max_y', max_y)
    #print('min_z', min_x)
    #print('maz_z', max_x)

    distance_between_y_extrema = np.abs(max_x - min_x)

    if distance_between_y_extrema > 0.1:
        end_point = int(extrema_elements[2])
        print("end point is in X", new_points[end_point])
    else:
        end_point = int(extrema_elements[0])
        print("end point is in Y", new_points[end_point])

    # sort the points 
    sorted_points = sort_points(new_points,end_point)

    # fit bezier curve 
    final_node_set = get_final_node_set(sorted_points,20)

    pose = Pose()
    pose_array = PoseArray()

    for i in range(N):
        pose = Pose()
        pose.position.x = final_node_set[i,0]
        pose.position.y = final_node_set[i,1]
        pose.position.z = final_node_set[i,2]
        pose.orientation.w = 1.0

        pose_array.poses.append(pose)   
    
    return ProcessPointCloudResponse(pose_array)
    
 
if __name__ == "__main__":
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('process_point_cloud', ProcessPointCloud, process_point_cloud)
    print("Process PointCloud Server is now running")
    rospy.spin()