#! /usr/bin/env python3

import rospy
import moveit_commander
import sys
import moveit_msgs.msg
import geometry_msgs.msg
from dual_robot_msgs.srv import GraspWire, GraspWireResponse
import colorama
from colorama import Fore
import copy


class RobotControl:
    def __init__(self):
        super(RobotControl, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot_control_server')
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()

        self.right_arm = moveit_commander.MoveGroupCommander("robot_a")
        self.left_arm = moveit_commander.MoveGroupCommander("robot_b")

        self.robot_control_service_ = rospy.Service("robot_control_service", GraspWire, self.control_callback)


    def control_callback(self,req):

        # insert Grasp Object in Scene 
        box_name = "grasp_object"

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = req.object_grasp_pose.position.x  
        box_pose.pose.position.y = req.object_grasp_pose.position.y  
        box_pose.pose.position.z = req.object_grasp_pose.position.z  
        self.scene.add_box(box_name, box_pose, size=(0.05, 0.05, 0.05))

        if(req.wire_grasping_robot == "left"):
            robot_grasp_object = self.right_arm
            robot_grasp_wire = self.left_arm
            print(Fore.BLUE + "UPDATE:= " + Fore.WHITE + "Assigning Robot Tasks")
            print(Fore.YELLOW + "     Left Robot is Grasping Wire")
            print(Fore.YELLOW + "     Right Robot is Grasping Object")
        else:
            robot_grasp_object = self.left_arm
            robot_grasp_wire = self.right_arm
            print(Fore.BLUE + "UPDATE:= " + Fore.WHITE + "Assigning Robot Tasks")
            print(Fore.YELLOW + "     Right Robot is Grasping Wire")
            print(Fore.YELLOW + "     Left Robot is Grasping Object")

        robot_grasp_object.set_planning_time(10.0)
        robot_grasp_wire.set_planning_time(10.0)

        # Move to the Computed Grasp Pose to grasp the wire 
        pose_target = geometry_msgs.msg.Pose()

        pose_target.orientation.w = req.wire_grasp_pose.orientation.w
        pose_target.orientation.x = req.wire_grasp_pose.orientation.x
        pose_target.orientation.y = req.wire_grasp_pose.orientation.y
        pose_target.orientation.z = req.wire_grasp_pose.orientation.z


        pose_target.position.x = req.wire_grasp_pose.position.x
        pose_target.position.y = req.wire_grasp_pose.position.y
        pose_target.position.z = req.wire_grasp_pose.position.z

        robot_grasp_wire.set_pose_target(pose_target)

        print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Execute Robot Control")
        print("     Executing action: grasping wire")
        plan1 = robot_grasp_wire.go(wait=True)
        robot_grasp_wire.stop()

        # Cartesian move to move wire in the direction of the pull vector
        pull_distance = 0.125
        waypoints = []

        wpose = robot_grasp_wire.get_current_pose().pose
        wpose.position.x += pull_distance*req.pull_vec.x  
        wpose.position.y += pull_distance*req.pull_vec.y 
        wpose.position.z += pull_distance*req.pull_vec.z
        waypoints.append(copy.deepcopy(wpose))

        (plan1, fraction) = robot_grasp_wire.compute_cartesian_path(waypoints, 0.01, 0.0)  

        print("     Executing action: moving wire")
        robot_grasp_wire.execute(plan1, wait=True)

        # Grasp Object 
        grasp_target = geometry_msgs.msg.Pose()
        grasp_target.orientation.w = 1.0
        grasp_target.position.x = req.object_grasp_pose.position.x - 0.075
        grasp_target.position.y = req.object_grasp_pose.position.y
        grasp_target.position.z = req.object_grasp_pose.position.z

        robot_grasp_object.set_pose_target(grasp_target)

        print("     Executing action: grasping object")
        plan1 = robot_grasp_object.go(wait=True)
        robot_grasp_object.stop()
        robot_grasp_object.clear_pose_targets()

        grasp_target.position.x = grasp_target.position.x + 0.05
        robot_grasp_object.set_pose_target(grasp_target)
        plan1 = robot_grasp_object.go(wait=True)
        robot_grasp_object.stop()
        robot_grasp_object.clear_pose_targets()

        self.scene.remove_world_object(box_name)

        return GraspWireResponse(status = True)


    



if __name__ == "__main__":
    robot_control = RobotControl()
    print(Fore.GREEN + "Robot Control Server is now running")
    rospy.spin()
    moveit_commander.roscpp_shutdown()