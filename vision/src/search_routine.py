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
import numpy as np 
import cam_calibration

import math

# Fix this import to something like `from dual_robot_control.robot_services import RobotControl`
import importlib.util
import sys
spec = importlib.util.spec_from_file_location("RobotControl", "/home/drojas/dlo_ws/src/wire_manipulation/manipulation/dual_robot_control/src/robot_services.py")
RC = importlib.util.module_from_spec(spec)
sys.modules["RobotControl"] = RC
spec.loader.exec_module(RC)

class SearchRoutine():
    def __init__(self, search_arm, grasp_arm) -> None:
        # Publisher for move flag
        self.move_flag_pub = rospy.Publisher("/move_flag", Bool, queue_size=1)
        # Robot Control
        self.robot_control = RC.RobotControl()
        # Arm assignments
        self.SEARCHING_ARM = search_arm
        self.GRASPING_ARM  = grasp_arm
        # Transform and frame lookup
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)


    def publish(self, value: Bool):
        self.move_flag_pub.publish(value)

    def search(self):
        # print(self.tfBuffer.lookup_transform('camera_color_optical_frame', 'mounted_aruco_0', rospy.Time(0), rospy.Duration(5)))
        SEARCHING = True
        while SEARCHING:
            self.robot_control.move_to_frame(self.SEARCHING_ARM, "search_target")
            # do a search with orientation rotation
            
            try:
                # Check if target ArUco has been found
                self.tfBuffer.lookup_transform('camera_color_optical_frame', 'arm_aruco_0', rospy.Time(0), rospy.Duration(5))
                SEARCHING = False # end search when aruco found
            except tf2_ros.LookupException:
                self.publish(True)
                
def main():
    rospy.init_node('search_routine')
    
    rate = rospy.Rate(60)
    searchRoutine = SearchRoutine("left", "right")

    searchRoutine.search()
    # while not rospy.is_shutdown():
    #     pass


#*** Node Starts Here ***#
if __name__ == "__main__":
    main()
