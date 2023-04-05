#!/usr/bin/env python3

#ROS
import rospy
import geometry_msgs.msg

# from dual_robot_msgs.srv import *
from time import sleep
import tf2_ros

class SlipDetect:
    def __init__(self, slip_delta):
        # Distance in meters of how far cable end ArUco must move to quantify as slip
        self.slip_delta = slip_delta 

    def monitor_aruco(self):
        pass
    # 

#*** Node Starts Here ***#
if __name__ == "__main__":
    rospy.init_node('slip_detect', anonymous=True)
