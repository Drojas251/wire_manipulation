#!/usr/bin/env python3

#ROS
import rospy
import geometry_msgs.msg

# from dual_robot_msgs.srv import *
from time import sleep
import tf2_ros

#*** Node Starts Here ***#
if __name__ == "__main__":
    rospy.init_node('slip_detect', anonymous=True)
