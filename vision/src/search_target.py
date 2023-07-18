#!/usr/bin/env python3
import rospy

# tf2 and Transformations
from tf.transformations import quaternion_from_euler
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo

# Camera capture
from cv_bridge import CvBridge,CvBridgeError
from collections import defaultdict

import cv2
import numpy as np 
import cam_calibration

import math

def transform_search_target(child_name: str, source: str, pos_adj, ori_adj) -> None:
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = source
    t.child_frame_id = "{}".format(child_name, source)

    t.transform.translation.x = ori_adj[0]
    t.transform.translation.y = ori_adj[1]
    t.transform.translation.z = ori_adj[2]
    
    q = quaternion_from_euler(pos_adj[0], pos_adj[1], pos_adj[2])

    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

def main():
    rospy.init_node('search_target')
    
    rate = rospy.Rate(60)

    # Define max and min for horizontal and vertical coordinates
    # +distance to panels, horizontal 0.325:-0.4, vertical -0.4:0.2
    MIN_X, MAX_X = -0.4, 0.3
    MIN_Y, MAX_Y = -0.4, 0.2

    # curr_x

    z, x, y, = 0.75, 0, -0.1 # initalize search target at middle origin position
    while not rospy.is_shutdown():
        # if message received to move:
        transform_search_target("search_target", "camera_link", [0, 0, 0], [z, x, y])
        
        # Adjust target for next search

        # furthest left right up down flags?

        rate.sleep()


#*** Node Starts Here ***#
if __name__ == "__main__":
    main()
