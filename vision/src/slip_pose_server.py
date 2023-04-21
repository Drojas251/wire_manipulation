#!/usr/bin/env python3

#ROS
import rospy
import geometry_msgs.msg
from std_msgs.msg import Bool
import sys
import moveit_commander
import math

from time import sleep
import tf2_ros

import time

class SlipPoseServer:
    def __init__(self, slip_delta):
        # Robot services to get poses
        moveit_commander.roscpp_initialize(sys.argv)
        self.right_arm = moveit_commander.MoveGroupCommander("a_bot_arm")
        self.left_arm = moveit_commander.MoveGroupCommander("b_bot_arm")

        # Offsets for calculating arm pose
        ARUCO_ARM_CALIBRATION = {"right": {"x":-0.05, "y":-0.035, "z":0.05}, 
                                 "left": {"x":-0.025, "y":0.075, "z":0.05}}

    def monitor_dist(self, end_grasping_arm: str):
        """
        Parameters:
            rate_input (float): hertz for rate to check Euclidean distance between marker and executing arm
            end_grasping_arm (str): string specifying whether left or right arm attempted at wire and slipped from
            slip_delta (float): distance in meters of how far marker and arm must be to quantify flag raise
        """
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        while not rospy.is_shutdown(): # loop takes ~0.099 - 0.1 seconds; 99.5ms
            try:
                self.end_pose = tfBuffer.lookup_transform("world", "aruco_0", rospy.Time())
                arm_pose = self.get_current_pose(end_grasping_arm)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("ERROR")
                continue
                
        def get_current_pose(self, robot_id:str):
            return self.left_arm.get_current_pose() if robot_id == "left" else self.right_arm.get_current_pose()
        
def main():
    rospy.init_node('slip_pose_server')

    marker_ = rospy.Publisher('/marker_array', MarkerArray, queue_size=1)
    s = rospy.Service('process_point_cloud', ProcessPointCloud, process_point_cloud)
    print("Process PointCloud Server is now running")

    rospy.spin()

#*** Node Starts Here ***#
if __name__ == "__main__":
    main()
