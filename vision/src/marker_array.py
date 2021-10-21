#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
import numpy as np 

class MarkerBasics(object):
    def __init__(self):
        self.marker_ = rospy.Publisher('/marker_array', MarkerArray, queue_size=1)
        self.rate = rospy.Rate(1)
        self.init_maker()

        

    def init_maker(self):

        self.pose_1 = Pose()
        self.pose_1.orientation.x = 0
        self.pose_1.orientation.y = 0
        self.pose_1.orientation.z = 0
        self.pose_1.orientation.w = 1
        self.pose_1.position.x = 1
        self.pose_1.position.y = 0
        self.pose_1.position.z = 1

        self.pose_2 = Pose()
        self.pose_2.orientation = self.pose_1.orientation
        self.pose_2.position.x = 1
        self.pose_2.position.y = 1
        self.pose_2.position.z = 1

        self.pose_3 = Pose()
        self.pose_3.orientation = self.pose_1.orientation
        self.pose_3.position.x = 0
        self.pose_3.position.y = 1
        self.pose_3.position.z = 1

        self.poses_ = PoseArray()
        self.poses_.header.frame_id = 'map'
        self.poses_.poses.append(self.pose_1)
        self.poses_.poses.append(self.pose_2)
        self.poses_.poses.append(self.pose_3)

        self.pose_ = Pose()
        count = 0
        self.markers = MarkerArray()


        for self.pose_ in self.poses_.poses:
        
            self.marker_object = Marker()
            self.marker_object.header.frame_id = 'map'
            self.marker_object.header.stamp = rospy.get_rostime()
            self.marker_object.ns = 'point'
            self.marker_object.id = count
            self.marker_object.type = Marker.SPHERE
            self.marker_object.action = Marker.ADD

            my_point = Point()
            my_point = self.pose_.position
            self.marker_object.pose.position = my_point

            self.marker_object.pose.orientation = self.pose_.orientation

            self.marker_object.scale.x = 1.0
            self.marker_object.scale.y = 1.0
            self.marker_object.scale.z = 1.0

            self.marker_object.color.r = 0.0
            self.marker_object.color.g = 0.0
            self.marker_object.color.b = 1.0

            self.marker_object.color.a = 1.0

            self.marker_object.lifetime = rospy.Duration(0)

            self.markers.markers.append(self.marker_object)

            count = count + 1

            




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