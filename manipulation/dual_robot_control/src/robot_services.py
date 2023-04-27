#! /usr/bin/env python3

import rospy
import moveit_commander
import sys
import moveit_msgs.msg
import geometry_msgs.msg
from dual_robot_msgs.srv import GraspWire, GraspWireResponse
from dual_robot_msgs.srv import GraspObject, GraspObjectResponse
from std_msgs.msg import String

from colorama import Fore
import copy
from scipy.spatial.transform import Rotation as R
import tf2_ros

class RobotControl:
    def __init__(self):
        super(RobotControl, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()

        self.right_arm = moveit_commander.MoveGroupCommander("a_bot_arm")
        self.left_arm = moveit_commander.MoveGroupCommander("b_bot_arm")
        self.right_gripper = moveit_commander.MoveGroupCommander("a_bot_gripper")
        self.left_gripper = moveit_commander.MoveGroupCommander("b_bot_gripper")

        # self.grasp_wire_service_ = rospy.Service("grasp_wire_service", GraspWire, self.grasp_wire_callback)
        # self.grasp_object_service_ = rospy.Service("grasp_object_service", GraspObject, self.grasp_object_callback)
        # self.sleep_arm_service = rospy.Service("sleep_arm_service", GraspObject, self.sleep_arm_callback)

        self.pre_grasp_offset = 0.05
        self.post_grasp_offset = 0.02
        self.num_of_grasp = 0

        self.grasp_object_name = ""

        # REAL
        self.LIVE_ARM_CALIBRATION = {"right": {"x":-0.05, "y":-0.035, "z":0.05}, 
                                 "left": {"x":-0.025, "y":0.075, "z":0.05}}
        # DEMO
        self.DEMO_ARM_CALIBRATION = {"right": {"x":-0.05, "y":-0.085, "z":0.00}, 
                                "left": {"x":-0.025, "y":0.075, "z":0.05}}

    def set_gripper(self, robot_id, pos):
        if robot_id == "left":
            self.left_gripper.set_named_target(pos)
            _, l_pos_gripper, _, _ = self.left_gripper.plan()
            self.left_gripper.execute(l_pos_gripper)
        elif robot_id == "right":
            self.right_gripper.set_named_target(pos)
            _, r_pos_gripper, _, _ = self.right_gripper.plan()
            self.right_gripper.execute(r_pos_gripper)

    def move_to_target(self, robot_id, target):
        if robot_id == "left":
            self.left_arm.set_named_target(target)
            l_error_code_val, l_plan, l_planning_time, l_error_code = self.left_arm.plan()
            if (l_error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS):
                self.left_arm.execute(l_plan)

        elif robot_id == "right":
            self.right_arm.set_named_target(target)
            r_error_code_val, r_plan, r_planning_time, r_error_code = self.right_arm.plan()
            if (r_error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS):
                self.right_arm.execute(r_plan)

    def move_to_pose(self, robot_id, pose):
        if robot_id == "left":
            self.left_arm.set_pose_target(pose)
            l_error_code_val, l_plan, l_planning_time, l_error_code = self.left_arm.plan()
            if (l_error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS):
                self.left_arm.execute(l_plan)
            else:
                print("Error code:", l_error_code_val)

        elif robot_id == "right":
            self.right_arm.set_pose_target(pose)
            r_error_code_val, r_plan, r_planning_time, r_error_code = self.right_arm.plan()
            if (r_error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS):
                self.right_arm.execute(r_plan)
            else:
                print("Error code:", r_error_code_val)

    def move_to_joint_goal(self, robot_id, joint_goal):
        if robot_id == "left":
            self.left_arm.set_joint_value_target(joint_goal)
            l_error_code_val, l_plan, l_planning_time, l_error_code = self.left_arm.plan()
            if (l_error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS):
                self.left_arm.execute(l_plan)

        elif robot_id == "right":
            self.right_arm.set_joint_value_target(joint_goal)
            r_error_code_val, r_plan, r_planning_time, r_error_code = self.right_arm.plan()
            if (r_error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS):
                self.right_arm.execute(r_plan)

    def move_to_aruco(self, robot_id: str, aruco_id: str):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        end_pose = None
        while not end_pose:
            try:
                end_pose = tfBuffer.lookup_transform("world", aruco_id, rospy.Time()).transform.translation
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

        demo = True
        ARM_ADJUSTMENTS = self.DEMO_ARM_CALIBRATION if demo else self.LIVE_ARM_CALIBRATION
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = end_pose.x + ARM_ADJUSTMENTS[robot_id]["x"]
        target_pose.position.y = end_pose.y + ARM_ADJUSTMENTS[robot_id]["y"]
        target_pose.position.z = end_pose.z + ARM_ADJUSTMENTS[robot_id]["z"]

        self.move_to_pose(robot_id, target_pose)

    def grasp_object(self, robot_id, object_grasp_pose):

        # insert Grasp Object in Scene 
        self.grasp_object_name = "grasp_object"
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = object_grasp_pose.position.x  
        box_pose.pose.position.y = object_grasp_pose.position.y  
        box_pose.pose.position.z = object_grasp_pose.position.z  
        self.scene.add_box(self.grasp_object_name, box_pose, size=(0.025, 0.025, 0.025))

        # Get robot move_group
        if(robot_id == "left"):
            robot = self.left_arm
        else:
            robot = self.right_arm

        # Set planning time
        robot.set_planning_time(5.0)

        ## GRASPING OBJECT
        status = False

        # Get possible grasp
        object_grasp_poses = self.create_grasp_repo(object_grasp_pose)

        # Test potential grasps until a valid motion plan is found
        for i in range(self.num_of_grasp):

            grasp_target = object_grasp_poses.poses[i] #get potential grasp pose

            q = grasp_target.orientation
            grasp_obj_rotm = R.from_quat([q.x,q.y,q.z,q.w])
            grasp_obj_rotm = grasp_obj_rotm.as_matrix()
            pre_grasp = self.pre_grasp_offset*grasp_obj_rotm[:,[0]] # get pre-grasp offset

            # Set pre-grasp position
            grasp_target.position.x = object_grasp_pose.position.x - float(pre_grasp[0])
            grasp_target.position.y = object_grasp_pose.position.y - float(pre_grasp[1])
            grasp_target.position.z = object_grasp_pose.position.z - float(pre_grasp[2])

            print("x", grasp_target.position.x)
            print("y", grasp_target.position.y)
            print("z", grasp_target.position.z)

            # Find valid grasp pose
            print("     Checking Grasp Orientaiton #" + str(i+1))
            robot.set_pose_target(grasp_target)
            error_code_val, plan1, planning_time, error_code = robot.plan()
            success = (error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS) # returns True if grasp plan is success
            
            if success == True:
                print(Fore.GREEN + "     Valid Grasp Found")
                print(grasp_target.orientation)

                print(Fore.WHITE + "     Executing action: grasping object")
                robot.execute(plan1) # execute grasp when a valid solution is found
                robot.clear_pose_targets()

                # Going in for Grasps
                grasp = self.post_grasp_offset*2*grasp_obj_rotm[:,[0]] # get pre-grasp offset
                grasp_target.position.x = object_grasp_pose.position.x + float(grasp[0])
                grasp_target.position.y = object_grasp_pose.position.y + float(grasp[1])
                grasp_target.position.z = object_grasp_pose.position.z + float(grasp[2])

                print("x", grasp_target.position.x)
                print("y", grasp_target.position.y)
                print("z", grasp_target.position.z)


                robot.set_pose_target(grasp_target)
                robot.go(wait=True)
                robot.stop()
                robot.clear_pose_targets()
                status = True

                break
            else:
                robot.clear_pose_targets()
                if i >= self.num_of_grasp:
                    print(Fore.RED + "NO SOLUTION FOUND")

        GraspObjectResponse(status = status)
        self.scene.remove_world_object(self.grasp_object_name)

    def grasp_wire(self, robot_id, wire_grasp_pose, pull_vec):

        # Determining which Robot will Grasp wire
        if(robot_id == "left"):
            robot = self.left_arm
        else:
            robot = self.right_arm

        # Set planning time
        robot.set_planning_time(5.0)

        ## Move to the Computed Grasp Pose to grasp the wire 
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = wire_grasp_pose.orientation.w
        pose_target.orientation.x = wire_grasp_pose.orientation.x
        pose_target.orientation.y = wire_grasp_pose.orientation.y
        pose_target.orientation.z = wire_grasp_pose.orientation.z

        # determine pre-grasp pose
        print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Execute Robot Control")
        print("     Executing action: pre-grasp of wire")

        q = pose_target.orientation
        grasp_obj_rotm = R.from_quat([q.x,q.y,q.z,q.w])
        grasp_obj_rotm = grasp_obj_rotm.as_matrix()
        pre_grasp = self.pre_grasp_offset*grasp_obj_rotm[:,[0]]

        pose_target.position.x = wire_grasp_pose.position.x - float(pre_grasp[0])
        pose_target.position.y = wire_grasp_pose.position.y - float(pre_grasp[1])
        pose_target.position.z = wire_grasp_pose.position.z - float(pre_grasp[2])

        robot.set_pose_target(pose_target)
        plan1 = robot.go(wait=True)
        robot.stop()

        # Grasp Wire
        print("     Executing action: grasping wire")

        grasp = self.post_grasp_offset*1.5*grasp_obj_rotm[:,[0]] # get pre-grasp offset
        pose_target.position.x = wire_grasp_pose.position.x + float(grasp[0])
        pose_target.position.y = wire_grasp_pose.position.y + float(grasp[1])
        pose_target.position.z = wire_grasp_pose.position.z + float(grasp[2])

        waypoints = []

        wpose = robot.get_current_pose().pose
        waypoints.append(pose_target)

        (plan1, fraction) = robot.compute_cartesian_path(waypoints, 0.01, 0.0)  
        
        print('fraction 1', fraction)

        if fraction < 1 :
            return GraspWireResponse(status = False)


        robot.execute(plan1, wait=True)

        # robot.set_pose_target(pose_target)
        # plan1 = robot.go(wait=True)
        # robot.stop()

        # Close gripper around wire
        # self.right_gripper.set_named_target("close")
        # _, r_open_gripper, _, _ = self.right_gripper.plan()
        # self.right_gripper.execute(r_open_gripper)

        # Cartesian move to move wire in the direction of the pull vector
        print("     Executing action: moving wire")
        pull_distance = 0.175
        waypoints = []

        wpose = robot.get_current_pose().pose
        wpose.position.x += pull_distance*pull_vec.x  
        wpose.position.y += pull_distance*pull_vec.y 
        wpose.position.z += pull_distance*pull_vec.z
        waypoints.append(copy.deepcopy(wpose))

        (plan1, fraction) = robot.compute_cartesian_path(waypoints, 0.01, 0.0)  

        print('fraction 2', fraction)

        if fraction < 1 :
            return GraspWireResponse(status = False)

        robot.execute(plan1, wait=True)

        return GraspWireResponse(status = True)

    def create_grasp_repo(self,object_grasp_pose):

        object_grasp_poses = geometry_msgs.msg.PoseArray()

        ## Setting up Grasps for pick and place
        grasps = [] 

        grasp1 = geometry_msgs.msg.Quaternion()
        grasp2 = geometry_msgs.msg.Quaternion()
        grasp3 = geometry_msgs.msg.Quaternion()
        grasp4 = geometry_msgs.msg.Quaternion()
        grasp5 = geometry_msgs.msg.Quaternion()
        grasp6 = geometry_msgs.msg.Quaternion()

        # Grasp Obj forward
        grasp1.w = 1.0
        grasps.append(grasp1)

        # pitch -30 degrees
        grasp2.x = 0.0
        grasp2.y = -0.258819
        grasp2.z = 0
        grasp2.w = 0.9659258
        grasps.append(grasp2)

        # pitch 30 degrees
        grasp3.x = 0.0
        grasp3.y = 0.258819
        grasp3.z = 0.0
        grasp3.w = 0.9659258
        grasps.append(grasp3)

        # roll 90 degrees
        grasp4.x = 0.7071068
        grasp4.y = 0.0
        grasp4.z = 0.0
        grasp4.w = 0.7071068
        grasps.append(grasp4)

        # roll 90 degrees pitch 30
        grasp5.x = 0.6830127
        grasp5.y = 0.1830127
        grasp5.z = 0.1830127
        grasp5.w = 0.6830127
        grasps.append(grasp5)

        # roll 90 degrees pitch 30
        grasp6.x = 0.6830127
        grasp6.y = -0.1830127
        grasp6.z = -0.1830127
        grasp6.w = 0.6830127
        grasps.append(grasp6)

        for i in range(len(grasps)):
            pose = geometry_msgs.msg.Pose()

            pose.position = object_grasp_pose.position
            pose.orientation = grasps[i]

            object_grasp_poses.poses.append(pose)
        
        self.num_of_grasp = len(grasps)

        return object_grasp_poses 

if __name__ == "__main__":
    rospy.init_node('robot_control_server')
    robot_control = RobotControl()

    print(Fore.GREEN + "Robot Control Server is now running")
    rospy.spin()
    moveit_commander.roscpp_shutdown()
