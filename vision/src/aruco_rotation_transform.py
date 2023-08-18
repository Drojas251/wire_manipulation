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

WIRE_OFFSET = 0.125#75

def transform_aruco_rotation(child_name: str, source: str, pos_adj, ori_adj) -> None:
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = source
    t.child_frame_id = "{}_{}".format(child_name, source)

    t.transform.translation.x = ori_adj[0] # Offset arm to right by value meters
    t.transform.translation.y = ori_adj[1]
    t.transform.translation.z = ori_adj[2] # Too close to wall, move back .05m

    q = quaternion_from_euler(pos_adj[0], pos_adj[1], pos_adj[2]) # pos_adj
    # q = quaternion_from_euler(-math.pi/2,math.pi/2,0) # match rotation of bot grippers

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
        ### THIS NODE IS FOR CONVERTING ARUCO POSITIONS TO WORLD THE ARMS CAN UNDERSTAND AND MOVE TO

        # transform_aruco_rotation("aruco_init", "mounted_aruco_0", [0, math.pi/2, 0], [WIRE_OFFSET, +0.05, 0.05]) # initial grasp parallel to marker, transform moved slightly up   
        # transform_aruco_rotation("aruco_retrieval", "mounted_aruco_0", [0, math.pi/2, 0], [WIRE_OFFSET, 0, 0.05])
        # transform_aruco_rotation("slip_enroute", "mounted_aruco_1", [math.pi/2, math.pi/2 + math.pi/4, math.pi/2], [-.05, -0.125, 0.15]) # hold at downward angle

        # transform_aruco_rotation("unplug_end", 0, [0, math.pi/2, 0], [WIRE_OFFSET+0.1, +0.05, 0.075])
        # transform_aruco_rotation("slip_enroute", 1, [math.pi/2, math.pi/2 + math.pi/6, math.pi/2], 
                                    # [-0.05, -0.075, 0.15]) # hold at downward angle
        # transform_aruco_rotation(1, [0, math.pi/2, 0], [-.05, 0.1, 0.1]) # no downward angle
        
        # transform_aruco_rotation("aruco_init", "arm_aruco_0", [0, math.pi/2, 0], [WIRE_OFFSET, +0.05, 0.05]) # initial grasp parallel to marker, transform moved slightly up   
        transform_aruco_rotation("adj", "mounted_aruco_0", [0, math.pi/2, 0], [WIRE_OFFSET, 0, 0.05])
        transform_aruco_rotation("adj", "arm_aruco_0", [0, math.pi/2, math.pi/4], [WIRE_OFFSET-0.05, 0.1, 0])
        
        # transform_aruco_rotation("slip_enroute", "arm_aruco_1", [math.pi/2, math.pi/2 + math.pi/4, math.pi/2], [-.05, -0.125, 0.15]) # hold at downward angle

        
        # OFFSETS: [-left/+right, -down/+up, -forward/+backward]
        rate.sleep()


#*** Node Starts Here ***#
if __name__ == "__main__":
    main()
