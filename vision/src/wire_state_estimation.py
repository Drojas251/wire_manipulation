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

def get_pc_data(topic):
    point_cloud_topic = topic
    data = rospy.wait_for_message(point_cloud_topic, PointCloud2)

    # Get point cloud into a [N,3] vector
    pc = ros_numpy.numpify(data) 
    points=np.zeros((len(pc)*len(pc[0]),3))
    count = 0
    for x in range(len(pc)):
        for y in range(len(pc[0])):
            points2 = pc[x,y]
            PP = [points2[0],points2[1],points2[2]]
            if not math.isnan(points2[1]):
                points[count] = [points2[0],points2[1],points2[2]]   
                count = count + 1    

    points_ = points[0:(count-1), :]

    kmeans = KMeans(n_clusters=20)
    kmeans.fit(points_)
    C = kmeans.cluster_centers_
    print(kmeans.cluster_centers_)
    print(kmeans.labels_[0])
    print(kmeans.labels_[200])
    print(kmeans.labels_[600])
    print(kmeans.labels_[1000])
    print(kmeans.labels_[2000])





    return C

#def get_nodes_from_pc(points):
    # k means clustering 

    



rospy.init_node('listener', anonymous=True)
topic = "/rscamera/depth/points"
points = get_pc_data(topic)
rospy.spin()