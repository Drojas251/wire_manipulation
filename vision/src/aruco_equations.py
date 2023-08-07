#!/usr/bin/env python3
from time import sleep
import rospy

# tf2 and Transformations
from tf.transformations import quaternion_from_euler
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool

# Camera capture
from cv_bridge import CvBridge,CvBridgeError
from collections import defaultdict

import cv2
import math
import numpy as np 
import cam_calibration

import math

class ArucoEquations():
    def __init__(self) -> None:
        # Store position and orientation values of search target
        self.x_pos = None
        self.y_pos = None
        self.z_pos = None

        self.x_ori = None
        self.y_ori = None
        self.z_ori = None
        self.w_ori = None

    def load_search_pose(self, child_name: str, source: str, pos_adj, ori_adj):
        # Get broadcasted source frame
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = source
        t.child_frame_id = "{}".format(child_name, source)

        # Perform any necessary transformations to the pose
        t.transform.translation.x = ori_adj[0]
        t.transform.translation.y = ori_adj[1]
        t.transform.translation.z = ori_adj[2]
        
        q = quaternion_from_euler(pos_adj[0], pos_adj[1], pos_adj[2])

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Save in class object
        self.x_pos = t.transform.translation.x
        self.y_pos = t.transform.translation.y
        self.z_pos = t.transform.translation.z

        self.x_ori = t.transform.translation.x
        self.y_ori = t.transform.translation.y
        self.z_ori = t.transform.translation.z
        self.w_ori = t.transform.translation.w

    def calculate_normal(self, dx = 0.1, dy = 0.1):
        # Use dx,dy of search algorithm; create dummy points to define planar at point
        A = [self.x_pos, self.y_pos, self.z_pos]
        B = [self.x_pos + dx, self.y_pos + dy/2, self.z_pos]
        C = [self.x_pos + dx/2, self.y_pos + dy, self.z_pos]

        # Get normal by finding cross product of AB and AC




def main():
    # calculate_normal("search_normal", "search_target", [0,0,0], [0,0,0])
    aruco_equations = ArucoEquations()
    aruco_equations.load_search_pose("search_normal", "search_target", [0,0,0], [0,0,0])
    
#*** Node Starts Here ***#
if __name__ == "__main__":
    main()
