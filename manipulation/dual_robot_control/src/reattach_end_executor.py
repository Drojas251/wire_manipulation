#!/usr/bin/env python3

#ROS
import rospy
import geometry_msgs.msg

# from dual_robot_msgs.srv import *
from time import sleep
import tf2_ros

from robot_services import RobotControl

#python
import numpy as np
import math
from colorama import Fore

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

    wire_grasping_robot = "left"
    object_grasping_robot = "right"
    status = robot_control.move_to_target(wire_grasping_robot, 'sleep')
    # status = robot_control.move_to_target(object_grasping_robot, 'sleep')

    # Open grippers on both arms
    # status = robot_control.set_gripper(wire_grasping_robot, "open")
    # status = robot_control.set_gripper(object_grasping_robot, "open")

## MY TESTING
    print("STATUS: Transforming ArUco Position")
    ### Buffer to find transform
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()
        try:
            trans = tfBuffer.lookup_transform("world", "aruco_1",rospy.Time())
            print(trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("error")
            continue

    # test_pose = geometry_msgs.msg.Pose()
    # test_pose.position.x = 0.5238098264109586
    # test_pose.position.y = 0.0018454038466918302
    # test_pose.position.z = 0.1731949099694735
    
    # test_pose.orientation.w = 1.0
    # status = robot_control.move_to_pose(wire_grasping_robot, test_pose)

## END

    #     #Task Executor
    #     if(wire_grasping_robot == "left"):
    #         object_grasping_robot = "right"
    #     else:
    #         object_grasping_robot = "left"

    #     # BEGIN ROUTINE
    #     # print(wire_grasping_robot, object_grasping_robot)
    #     # Begin both arms in sleep
    #     status = robot_control.move_to_target(wire_grasping_robot, 'sleep')
    #     status = robot_control.move_to_target(object_grasping_robot, 'sleep')

    #     # Set both arms to a ready state
    #     status = robot_control.move_to_target(wire_grasping_robot, 'ready')
    #     status = robot_control.move_to_target(object_grasping_robot, 'ready')
    #     # Open grippers on both arms
    #     status = robot_control.set_gripper(wire_grasping_robot, "open")
    #     status = robot_control.set_gripper(object_grasping_robot, "open")
        
    #rospy.spin()
    