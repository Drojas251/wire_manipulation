#!/usr/bin/env python3
from numpy.core.fromnumeric import shape, size
import rospy
#import pcl
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import math

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from sklearn.cluster import KMeans

def callback(data):
    pc = ros_numpy.numpify(data)
    points=np.zeros((len(pc)*len(pc[0]),3))
    count = 0

    # format the processed pointcloud into a N x 3 numpy array
    for x in range(len(pc)):
        for y in range(len(pc[0])):
            points2 = pc[x,y]
            PP = [points2[0],points2[1],points2[2]]
            if not math.isnan(points2[1]):
                points[count] = [points2[0],points2[1],points2[2]]   
                count = count + 1    

    # After "count" all elements should be zero. Extract only non-zero elements
    points_ = points[0:(count-1), :]

    # cluster the point cloud data into N = 20 nodes
    kmeans = KMeans(n_clusters=20)
    kmeans.fit(points_)
    #print(kmeans.labels_)
    C = kmeans.cluster_centers_
    
    # Find outliers 
    outliers = []
    threshold = 0.1 # a point whos closest neighbor is greater than 0.1m away is considered an outlier 

    for k in range(20):
        min_dist = 5
        for j in range(20):
            if k != j:
                dist = ((C[k,0] - C[j,0])**2 + (C[k,1] - C[j,1])**2 + (C[k,2] - C[j,2])**2 )**0.5
                if dist < min_dist:
                    min_dist = dist
            
        # check outlier condition    
        if min_dist > threshold: 
            outliers.append(k)
        
    """ # define pose array
    poses_ = PoseArray()
    poses_.header.frame_id = 'camera_link'

    for i in range(20):
        pose = Pose()
        pose.position.x = C[i,0]
        pose.position.y = C[i,1]
        pose.position.z = C[i,2]
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        
        poses_.poses.append(pose)
    """
    count = 0
    markers = MarkerArray()

    for j in range(20):

        marker_object = Marker()
        marker_object.header.frame_id = 'camera_color_optical_frame'
        marker_object.header.stamp = rospy.get_rostime()
        marker_object.ns = 'point'
        marker_object.id = count
        marker_object.type = Marker.SPHERE
        marker_object.action = Marker.ADD

        my_point = Point()
        my_point.x = C[j,0]
        my_point.y = C[j,1]
        my_point.z = C[j,2]
        
        marker_object.pose.position = my_point

        marker_object.pose.orientation.w = 1
        marker_object.pose.orientation.x = 0
        marker_object.pose.orientation.y = 0
        marker_object.pose.orientation.z = 0

        marker_object.scale.x = 0.085
        marker_object.scale.y = 0.085
        marker_object.scale.z = 0.085

        # color the outlier in blue
        # TO DO: Remove outlier from data set
        for out in outliers:
            if marker_object.id == out:
                marker_object.color.r = 0.0
                marker_object.color.g = 0.0
                marker_object.color.b = 1.0
                break

            else:
                marker_object.color.r = 1.0
                marker_object.color.g = 0.0
                marker_object.color.b = 0.0

        marker_object.color.a = 1.0
        marker_object.lifetime = rospy.Duration(0)

        markers.markers.append(marker_object)

        count = count + 1

    marker_.publish(markers)    


rospy.init_node('listener', anonymous=True)
rospy.Subscriber("/rscamera/depth/points", PointCloud2, callback)
marker_ = rospy.Publisher('/marker_array', MarkerArray, queue_size=1)
rospy.spin()