#!/usr/bin/env python3

#ROS
import numpy as np
import rospy
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Bool, String
from colorama import Fore
from std_srvs.srv import SetBool

# from dual_robot_msgs.srv import *
from time import sleep
from robot_services import RobotControl

# Temp solution - Fix this import to something like `from dual_robot_control.robot_services import RobotControl`
import importlib.util
import sys
spec = importlib.util.spec_from_file_location("SearchRoutine", "/home/drojas/dlo_ws/src/wire_manipulation/vision/src/search_routine.py")
SC = importlib.util.module_from_spec(spec)
sys.modules["RobotControl"] = SC
spec.loader.exec_module(SC)

# Client call to swap ML camera specification 
def set_cam_spec_service(value : Bool):
     rospy.wait_for_service("/set_cam_spec")
     try:
         set_cam_spec = rospy.ServiceProxy('/set_cam_spec', SetBool)
         response = set_cam_spec(value)
         return response.success, response.message
     except rospy.ServiceException as e:
         print("Service call failed: %s"%e)

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

    print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Initiating Robots ")
    robot_control = RobotControl()

    GRASPING_ARM    = "right"
    GRASPING_ARM_ID = "a_bot_arm" if GRASPING_ARM == "right" else "b_bot_arm"

    SEARCHING_ARM   = "left"
    SEARCHING_ARM_ID = "a_bot_arm" if SEARCHING_ARM == "right" else "b_bot_arm"

    # status = robot_control.move_to_frame(GRASPING_ARM, "prepose_grasp_mounted_cam")
    # status = robot_control.move_to_frame(GRASPING_ARM, "perp_line_grasp_mounted_cam")

    joint_goal0 = [41, 1, -2, 90, -43, 3] # start
    status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal0])

    # SEARCH TESTING
    # Search algorithm loop would look like:
    # 1. Identify no ArUco tags with rear cam
    # 2. Move searching arm to first search target
    # 3. Conduct search on search target
    # 4. If nothing found, move search target and arm and loop

    # status = robot_control.move_to_frame(GRASPING_ARM, "adj_arm_aruco_0")

    # status = robot_control.move_to_frame(GRASPING_ARM, "prepose_grasp_mounted_cam")
    # status = robot_control.move_to_frame(GRASPING_ARM, "perp_line_grasp_mounted_cam")

    # joint_goal_arm_cam = [-45, -79, 42, 10, 67, -14]
    # status = robot_control.move_to_joint_goal(SEARCHING_ARM, [x * np.pi / 180 for x in joint_goal_arm_cam])

    # sleep(5)

    # success, message = set_cam_spec_service(True) # swap back to arm only

    # status = robot_control.move_to_frame(GRASPING_ARM, "prepose_grasp_arm_cam")
    # status = robot_control.move_to_frame(GRASPING_ARM, "perp_line_grasp_arm_cam")
    
    # print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Initiating Robots ")
    # # status = robot_control.move_to_frame(SEARCHING_ARM, "search_target")
    # print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Attempt grabbing ArUco from rear camera view")
    # status = robot_control.move_to_frame(GRASPING_ARM, "adj_mounted_aruco_0")
    # if status == None:
    #     print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Rear camera view attempt failed, initiate search routine")
    #     # Initiate search algorithm
    #     print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Initiate search routine")
    #     searchRoutine = SC.SearchRoutine("left", "right")
    #     search_result = searchRoutine.search(True)

    #     if search_result:
    #         print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Search successful, send grasping arm for retrieval")
    #         # send right arm to aruco left arm found
    #         sleep(2.5)
    #         status = robot_control.move_to_frame(GRASPING_ARM, "usb-crotation")
    # else:
    #     print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Rear camera view attempt successful")
    