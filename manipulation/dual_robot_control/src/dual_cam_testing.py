#!/usr/bin/env python3

#ROS
import numpy as np
import rospy
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Bool

# from dual_robot_msgs.srv import *
from time import sleep
from robot_services import RobotControl

# Client call to grasp and move wire 
def grasp_wire(robot_,wire_grasp_pose,pull_vec):
     rospy.wait_for_service('grasp_wire_service')
     try:
         grasp_wire_input = rospy.ServiceProxy('grasp_wire_service', GraspWire)
         response = grasp_wire_input(robot_,wire_grasp_pose,pull_vec)
         return response
     except rospy.ServiceException as e:
         print("Service call failed: %s"%e)

# Client call to grasp target object
def grasp_target(robot_,object_grasp_pose):
     rospy.wait_for_service('grasp_object_service')
     try:
         grasp_object_input = rospy.ServiceProxy('grasp_object_service', GraspObject)
         response = grasp_object_input(robot_,object_grasp_pose)
         return response
     except rospy.ServiceException as e:
         print("Service call failed: %s"%e)

# Client call to sleep specified arm
def sleep_arm(robot_):
    rospy.wait_for_service('sleep_arm_service')
    
    sleep_arm_input = rospy.ServiceProxy('/sleep_arm_service', GraspObject)
    req = GraspObjectRequest()
    pose = geometry_msgs.msg.Pose()
    pose.position = geometry_msgs.msg.Point(0,0,0)
    pose.orientation = geometry_msgs.msg.Quaternion(0,0,0,0)
    req.robot = 'left'
    req.object_grasp_pose = pose
    response = sleep_arm_input(req)

#*** Node Starts Here ***#
if __name__ == "__main__":
    rospy.init_node('listener', anonymous=True)
    robot_control = RobotControl()

    GRASPING_ARM    = "right"
    GRASPING_ARM_ID = "a_bot_arm" if GRASPING_ARM == "right" else "b_bot_arm"

    sleep(2)
    ##  Move grasping arm to wire end found via a_bot mounted cam
    print("STATUS: Move to arm camera aruco pose")
    status = robot_control.move_to_aruco(GRASPING_ARM, "search_target")
    # print(status)
    # sleep(5)
    # print("STATUS: Sleep reset")
    # status = robot_control.move_to_target(GRASPING_ARM, 'sleep')
    # print("STATUS: Move to mounted camera aruco pose")
    # status = robot_control.move_to_aruco(GRASPING_ARM, "aruco_retrieval_mounted_aruco_0")
    # print(status)
    # sleep(5)
    # status = robot_control.move_to_target(GRASPING_ARM, 'sleep')
    # rospy.spin()
    