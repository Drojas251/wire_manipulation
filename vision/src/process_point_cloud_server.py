#!/usr/bin/env python3
import rospy
import ros_numpy
import numpy as np
from numpy import linalg as la
import math

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from sklearn.cluster import KMeans
from wire_modeling.wire_sim import *
from wire_modeling.wire_grasp_toolbox import WireGraspToolbox, rotm
from wire_modeling_msgs.srv import ProcessPointCloud, ProcessPointCloudResponse



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
    Wire = WireGraspToolbox(sorted_points)
    
    #curve_set = Bezier.Curve(t_points, sorted_points)
    curve_set = Wire.BCurve()

    final_node_set = np.zeros((N,3))

    for i in range(N):
        final_node_set[[i],:] = curve_set[[i*5],:]

    return final_node_set 

def get_wire_length(points):
    length = 0
    for i in range(len(points)-1):
        length = length + ((points[i,0] - points[i+1,0])**2 + (points[i,1] - points[i+1,1])**2 + (points[i,2] - points[i+1,2])**2 )**0.5

    return length


def process_point_cloud(req):
    # points are in camera_color_optical_frame
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


    #TO DO:: Automate this with tf2
    
    # Transform the points from camera_color_optical_frame to camera_frame 
    # rotation about x -90 
    # rotation about y 90
    # Transformation from camera frame to world is pure translation 
    #-0.2286 0 0.4318

    angle = math.pi/2
    ROT = rotm(-angle,angle,0)

    for i in range(len(new_points)):
        new_points[[i],:] = np.transpose(ROT@np.transpose(new_points[[i],:])) + np.array((-0.3556,0.015,0.4064))

    # check the extreme points ( maz min x,y,z)
    # find the element with the most extreme values
    # ex. element 6 might hold the max x and min z. Likley an end point
    # use this point as a starting point
    min_y = max_y = new_points[0,1]
    min_z = max_z = new_points[0,2]
    extrema_elements = np.zeros(4) # [min_y, max_y, min_z, max_z]

    for j in range(len(new_points)):
        if new_points[j,1] <= min_y:
            min_y = new_points[j,1]
            extrema_elements[0] = int(j)
        if new_points[j,1] >= max_y:
            max_y = new_points[j,1]
            extrema_elements[1] = int(j)
        if new_points[j,2] <= min_z:
            min_z = new_points[j,2]
            extrema_elements[2] = int(j)
        if new_points[j,2] >= max_z:
            max_z = new_points[j,2]
            extrema_elements[3] = int(j)

    distance_between_y_extrema = np.abs(max_y - min_y)
    distance_between_z_extrema = np.abs(max_z - min_z)

    if distance_between_z_extrema > distance_between_y_extrema:
        end_point = int(extrema_elements[2])
        print("end point is in Z", new_points[end_point])
        wire_class = "type1" # wire is mostly verticle
    else:
        end_point = int(extrema_elements[0])
        print("end point is in Y", new_points[end_point])
        wire_class = "type2" # wire is mostyl horizontal 

    # sort the points 
    sorted_points = sort_points(new_points,end_point)


    # fit bezier curve 
    final_node_set = get_final_node_set(sorted_points,20)


    pose_array = PoseArray()
    markers = MarkerArray()
    raw_points = PoseArray()

    for j in range(len(sorted_points)):
        pose = Pose()
        pose.position.x = sorted_points[j,0]
        pose.position.y = sorted_points[j,1]
        pose.position.z = sorted_points[j,2]
        pose.orientation.w = 1.0

        raw_points.poses.append(pose)


    for i in range(N):
        pose = Pose()
        pose.position.x = final_node_set[i,0]
        pose.position.y = final_node_set[i,1]
        pose.position.z = final_node_set[i,2]
        pose.orientation.w = 1.0

        pose_array.poses.append(pose)  

        # Visualization 
        marker_object = Marker()
        marker_object.header.frame_id = 'world'
        marker_object.header.stamp = rospy.get_rostime()
        marker_object.ns = 'point'
        marker_object.id = i
        marker_object.type = Marker.SPHERE
        marker_object.action = Marker.ADD

        
        marker_object.pose.position = pose.position
        marker_object.pose.orientation = pose.orientation

        marker_object.scale.x = 0.025
        marker_object.scale.y = 0.025
        marker_object.scale.z = 0.025

        if i == 0:
            marker_object.color.r = 0.0
            marker_object.color.g = 1.0
            marker_object.color.b = 0.0
        else:
            marker_object.color.r = 1.0
            marker_object.color.g = 0.0
            marker_object.color.b = 0.0

        marker_object.color.a = 1.0
        marker_object.lifetime = rospy.Duration(0)

        markers.markers.append(marker_object)

    marker_.publish(markers)
    wire_length = get_wire_length(final_node_set)

    
    return ProcessPointCloudResponse(pose = pose_array, wire_length = wire_length, raw_points = raw_points, wire_class = wire_class)
    
 
if __name__ == "__main__":
    rospy.init_node('process_point_cloud_server')
    marker_ = rospy.Publisher('/marker_array', MarkerArray, queue_size=1)
    s = rospy.Service('process_point_cloud', ProcessPointCloud, process_point_cloud)
    print("Process PointCloud Server is now running")
    rospy.spin()