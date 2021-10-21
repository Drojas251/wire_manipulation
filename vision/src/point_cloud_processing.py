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

def callback(data):
    pc = ros_numpy.numpify(data)
    points=np.zeros((len(pc)*len(pc[0]),3))
    count = 0
    

    pose_1 = Pose()
    pose_1.orientation.x = 0
    pose_1.orientation.y = 0
    pose_1.orientation.z = 0
    pose_1.orientation.w = 1

    pose_2 = Pose()
    pose_2.orientation = pose_1.orientation

    pose_3 = Pose()
    pose_3.orientation = pose_1.orientation

    pose_4 = Pose()
    pose_4.orientation = pose_1.orientation

    pose_5 = Pose()
    pose_5.orientation = pose_1.orientation




    for x in range(len(pc)):
        for y in range(len(pc[0])):
            points2 = pc[x,y]
            PP = [points2[0],points2[1],points2[2]]
            if not math.isnan(points2[1]):
                points[count] = [points2[0],points2[1],points2[2]]   
                count = count + 1    

    space = (count-10)/4
    space = round(space)

    num1 = 0
    num2 = space
    num3 = 2*space
    num4 = 3*space
    num5 = 4*space

    pose_1.position.x = points[num1,0]
    pose_1.position.y = points[num1,1]
    pose_1.position.z = points[num1,2]

    pose_2.position.x = points[num2,0]
    pose_2.position.y = points[num2,1]
    pose_2.position.z = points[num2,2] 

    pose_3.position.x = points[num3,0]
    pose_3.position.y = points[num3,1]
    pose_3.position.z = points[num3,2] 

    pose_4.position.x = points[num4,0]
    pose_4.position.y = points[num4,1]
    pose_4.position.z = points[num4,2] 

    pose_5.position.x = points[num5,0]
    pose_5.position.y = points[num5,1]
    pose_5.position.z = points[num5,2] 

    poses_ = PoseArray()
    poses_.header.frame_id = 'camera_link'
    poses_.poses.append(pose_1)
    poses_.poses.append(pose_2)
    poses_.poses.append(pose_3)
    poses_.poses.append(pose_4)
    poses_.poses.append(pose_5)

    pose_ = Pose()
    count = 0
    markers = MarkerArray()


    for pose_ in poses_.poses:

        marker_object = Marker()
        marker_object.header.frame_id = 'camera_link'
        marker_object.header.stamp = rospy.get_rostime()
        marker_object.ns = 'point'
        marker_object.id = count
        marker_object.type = Marker.SPHERE
        marker_object.action = Marker.ADD

        my_point = Point()
        my_point = pose_.position
        marker_object.pose.position = my_point

        marker_object.pose.orientation = pose_.orientation

        marker_object.scale.x = 0.0085
        marker_object.scale.y = 0.0085
        marker_object.scale.z = 0.0085

        marker_object.color.r = 1.0
        marker_object.color.g = 0.0
        marker_object.color.b = 0.0

        marker_object.color.a = 1.0

        marker_object.lifetime = rospy.Duration(0)

        markers.markers.append(marker_object)

        count = count + 1
    
    marker_.publish(markers)    

            



    #points[:,0]=pc['x']
    #points[:,1]=pc['y']
    #points[:,2]=pc['z']
    #p = pcl.PointCloud(np.array(points, dtype=np.float32))
    

rospy.init_node('listener', anonymous=True)
rospy.Subscriber("/rscamera/depth/points", PointCloud2, callback)
marker_ = rospy.Publisher('/marker_array', MarkerArray, queue_size=1)
rospy.spin()