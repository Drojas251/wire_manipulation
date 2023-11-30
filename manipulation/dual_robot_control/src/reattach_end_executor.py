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
    arm_ids = ["left","right"]
    
    # Send object grasp robot to pre-grasp, encountering wire
    joint_goal0 = [41, 1, -2, 90, -43, 3] # start
    joint_goal0_5 = [57, -14, 14, 89, -60, 4]
    joint_goal1 = [21, -13, 15, 81, -19, 9] # unplug
    joint_goal2 = [36, 13, -41, 153, -90, -53] # slip enroute angled down
    joint_goal3 = [47, 2, -14, -79, 52, -16] # final reattach
    
    ### START ROUTINE for full demonstration at annual review
    ##  Initialize arms; Sleep, open grippers, and ready pose
    for arm in arm_ids: 
        status = robot_control.move_to_target(arm, 'sleep')
        status = robot_control.set_gripper(arm, "open")

    ##  Move grasping arm to wire end and unplug
    status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal0_5])
    status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal0])
    ## Grip and unplug
    status = robot_control.set_gripper(GRASPING_ARM, "close")
    status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal1])
    status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal0])
    status = robot_control.set_gripper(GRASPING_ARM, "open")
                       
    ### START SCENARIO C4 as soon as wire is grasped, earliest wire could slip
    print("STATUS: Begin Scenario C4")
    sleep(3)
    a1_commands = [ # Commands from A1
        # Send to stowing point, engineering slip
        "status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal2])",
    ]
    for command in a1_commands:
        exec(command)
        slip_flag = rospy.wait_for_message("{}_marker_delta_flag".format(GRASPING_ARM_ID), Bool)
        if (slip_flag): # If slip detected, move arm to retrieve wire
            print("STATUS: Slip detected, initiate retrieval")
            sleep(10) # wait 5 real time seconds for slipped wire to settle
            status = robot_control.move_to_target(GRASPING_ARM, 'sleep')
            status = robot_control.set_gripper(GRASPING_ARM, "open")

            status = robot_control.move_to_frame(GRASPING_ARM, "prepose_grasp_mounted_cam")
            status = robot_control.move_to_frame(GRASPING_ARM, "perp_line_grasp_mounted_cam")
            # status = robot_control.move_to_frame(GRASPING_ARM, "final_prepose_mounted_cam")
            # status = robot_control.move_to_frame(GRASPING_ARM, "final_pose_mounted_cam")

            status = robot_control.set_gripper(GRASPING_ARM, "close")
            
            status = robot_control.move_to_joint_goal(GRASPING_ARM, [x * np.pi / 180 for x in joint_goal3])
    ### END SCENARIO C4

    # rospy.spin()
    