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

def transform_aruco_rotation(aruco_id):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "aruco_{}".format(aruco_id)
    t.child_frame_id = "aruco_wire_rotation_{}".format(aruco_id)

    tf_conversions

    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0

    q = quaternion_from_euler(-math.pi/2,math.pi/2,0)

    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

def main():
    rospy.init_node('aruco_rotation_transform')

    # print("Aruco Pose Server is now running")
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        transform_aruco_rotation(0)
        rate.sleep()


#*** Node Starts Here ***#
if __name__ == "__main__":
    main()
