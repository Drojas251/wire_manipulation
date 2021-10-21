#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
import numpy as np 

class MarkerBasics(object):
    def __init__(self):
        self.marker_ = rospy.Publisher('/marker', Marker, queue_size=1)
        self.rate = rospy.Rate(1)
        self.init_maker(index=0, z_val=0)

    def init_maker(self, index=0, z_val=0):
        self.marker_object = Marker()
        self.marker_object.header.frame_id = 'map'
        self.marker_object.header.stamp = rospy.get_rostime()
        self.marker_object.ns = 'point'
        self.marker_object.id = index
        self.marker_object.type = Marker.SPHERE
        self.marker_object.action = Marker.ADD

        my_point = Point()
        my_point.z = z_val
        self.marker_object.pose.position = my_point

        self.marker_object.pose.orientation.x = 0
        self.marker_object.pose.orientation.y = 0
        self.marker_object.pose.orientation.z = 0
        self.marker_object.pose.orientation.w = 1.0

        self.marker_object.scale.x = 1.0
        self.marker_object.scale.y = 1.0
        self.marker_object.scale.z = 1.0

        self.marker_object.color.r = 0.0
        self.marker_object.color.g = 0.0
        self.marker_object.color.b = 1.0

        self.marker_object.color.a = 1.0

        self.marker_object.lifetime = rospy.Duration(0)

    def start(self):
        while not rospy.is_shutdown():
            self.marker_.publish(self.marker_object)
            self.rate.sleep()


if __name__=='__main__':
    rospy.init_node('marker_test',anonymous=True)
    marker_basics_object = MarkerBasics()
    try:
        marker_basics_object.start()
    except rospy.ROSInternalException:
        pass