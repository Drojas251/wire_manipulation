#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
import numpy as np 
from wire_modeling.wire_grasp_toolbox import WireGraspToolbox

class MarkerBasics(object):
    def __init__(self):
        self.marker_ = rospy.Publisher('/marker_array', MarkerArray, queue_size=1)
        self.rate = rospy.Rate(1)
        self.init_maker()


    def get_final_node_set(self,sorted_points,N):
        # N = number of nodes
        Wire = WireGraspToolbox(sorted_points)
        
        #curve_set = Bezier.Curve(t_points, sorted_points)
        curve_set = Wire.BCurve()

        final_node_set = np.zeros((N,3))

        for i in range(N):
            final_node_set[[i],:] = curve_set[[i*5],:]

        return final_node_set 

    def init_maker(self):


        print("Creating Data")
        x = 0.39
        raw_data = np.array([[x,-0.25,0.08],[x,-0.2,0.2],[x,-0.15,0.4],[x,-.05,0.48]]) #Wire1
        #raw_data = np.array([[x,-0.2,0.13],[x,-0.1,0.2],[x,0.0,0.2],[x,.1,0.15]]) # Wire2
        #raw_data = np.array([[x,0,0.0],[x,0.02,0.15],[x,0.05,0.3],[x,0.03,0.45]]) # Wire3
        points = self.get_final_node_set(raw_data,20)

        wire = np.zeros((3,20))

        for i in range(20):
            wire[:,[i]] = np.transpose(points[[i],:])


        # visualize 
        pose_array = PoseArray()
        self.markers = MarkerArray()


        for i in range(20):
            pose = Pose()
            pose.position.x = points[i,0]
            pose.position.y = points[i,1]
            pose.position.z = points[i,2]
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

            self.markers.markers.append(marker_object)

    
    def start(self):
        while not rospy.is_shutdown():
            self.marker_.publish(self.markers)
            self.rate.sleep()


if __name__=='__main__':
    rospy.init_node('marker_test',anonymous=True)
    marker_basics_object = MarkerBasics()
    try:
        marker_basics_object.start()
    except rospy.ROSInternalException:
        pass