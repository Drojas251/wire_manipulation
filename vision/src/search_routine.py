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
        self.move_flag_pos_pub  = rospy.Publisher("/move_flag_pos",  Bool, queue_size=10)
        self.move_flag_ori_pub  = rospy.Publisher("/move_flag_ori",  Bool, queue_size=10)
        # Robot Control
        self.robot_control = RC.RobotControl()
        # Arm assignments
        self.SEARCHING_ARM = search_arm
        self.GRASPING_ARM  = grasp_arm
        # Transform and frame lookup
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def search(self):
        # print(self.tfBuffer.lookup_transform('camera_color_optical_frame', 'mounted_aruco_0', rospy.Time(0), rospy.Duration(5)))
        SEARCHING = True
        TAG_FOUND = False
        while SEARCHING:
            # Move to search target position
            self.robot_control.move_to_frame(self.SEARCHING_ARM, "search_target")
            # check flat parallel orientation
            try:
                # Check if target ArUco has been found
                print("TRY 1: Scan parallel for subpoint")
                self.tfBuffer.lookup_transform('camera_color_optical_frame', 'arm_aruco_0', rospy.Time(0), rospy.Duration(5))
            except tf2_ros.LookupException: # Begin checking each subpoints in a spiral node position
                print("EXCEPT 1: No parallel found, move to start searching subpoints")
                
                # Scan points at the current node
                for i in range(8): # 8 positions of plane
                    try:
                        print("TRY 2: Move to subpoint attempt", i)
                        # Move search target to subpoint
                        self.move_flag_ori_pub.publish(True) # move orientation along 9 points

                        # Wait for publish from search target before moving?
                        # Move arm to subpoint 
                        self.robot_control.move_to_frame(self.SEARCHING_ARM, "search_target")

                        print("TRY 2: Scan subpoint")
                        # Check if target ArUco has been found
                        self.tfBuffer.lookup_transform('camera_color_optical_frame', 'arm_aruco_0', rospy.Time(0), rospy.Duration(5))
                        # Throws error if no lookup found
                        SEARCHING = False # end search when aruco found
                        TAG_FOUND = True
                        break
                    except tf2_ros.LookupException:
                        pass
                        # print("EXCEPT 2: Move to next subpoint at node")
                        # self.move_flag_ori_pub.publish(True) # move orientation along 9 points
            print("STATUS: Move to next node")
            # This publish moves to next node in spiral
            self.move_flag_pos_pub.publish(True)
        return TAG_FOUND
                
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
