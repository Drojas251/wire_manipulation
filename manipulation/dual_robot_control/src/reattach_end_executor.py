#!/usr/bin/env python3

#ROS
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
    arm_ids = ["left","right"]
    
    ### START ROUTINE
    ##  Initialize arms; Sleep, open grippers, and ready pose
    for arm in arm_ids: 
        status = robot_control.move_to_target(arm, 'sleep')
        status = robot_control.set_gripper(arm, "open")
        status = robot_control.move_to_target(arm, 'ready')

    ##  Move grasping arm to wire end
    #   Rotate gripper to accomodate wire
    # joint_goal = [0, -75, 78, 0, 43, 90]
    # joint_goal = [x * np.pi / 180 for x in joint_goal]
    # status = robot_control.move_to_joint_goal(GRASPING_ARM, joint_goal)
    #   Move grasping arm to ArUco marker
    status = robot_control.move_to_aruco(GRASPING_ARM, "aruco_0")
    #   Close gripper of grasping arm around wire
    status = robot_control.set_gripper(GRASPING_ARM, "close")
                                         
    ## START SCENARIO C4 as soon as wire is grasped, earliest wire could slip
    print("STATUS: Begin Scenario C4")

    dummy_move = geometry_msgs.msg.Pose()
    dummy_move.position.x =  0.4751494271654885
    dummy_move.position.y = -0.07983718963975926
    dummy_move.position.z =  0.17235676317975507 + 0.2

    a1_commands = [ # Commands from A1
        # Send to dummy plate
        "status = robot_control.move_to_pose(GRASPING_ARM, dummy_move)",
    ]
    for command in a1_commands:
        exec(command)
        slip_flag = rospy.wait_for_message("{}_marker_delta_flag".format(GRASPING_ARM_ID), Bool)
        if (slip_flag): # If slip detected, move arm to retrieve wire
            print("STATUS: Slip detected, initiate retrieval")
            status = robot_control.set_gripper(GRASPING_ARM, "open")
            sleep(5) # wait 5 real time seconds for slipped wire to settle

            status = robot_control.move_to_aruco(GRASPING_ARM, "aruco_0")
            status = robot_control.set_gripper(GRASPING_ARM, "close")
    ## END SCENARIO C4

    # rospy.spin()
    