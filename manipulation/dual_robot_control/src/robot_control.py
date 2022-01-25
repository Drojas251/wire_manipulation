#! /usr/bin/env python3


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


def move_to_pose(x,y,z,robot):

    pose_target3 = geometry_msgs.msg.Pose()
    pose_target3.orientation.w = 1.0
    pose_target3.position.x = x
    pose_target3.position.y = y
    pose_target3.position.z = z

    if robot == "robot_a":
        robot_a_group.set_pose_target(pose_target3)
        print("Executing Move: Position3")
        plan1 = robot_a_group.go(wait=True)
        print("plan and execute")
        robot_a_group.stop()
        print("stop")
        robot_a_group.clear_pose_targets()
        print("clear outputs")
        variable = robot_a_group.get_current_pose()
        print (variable.pose)

    if robot == "robot_b":
        robot_b_group.set_pose_target(pose_target3)
        print("Executing Move: Position3")
        plan2 = robot_b_group.go(wait=True)
        print("plan and execute")
        robot_b_group.stop()
        print("stop")
        robot_b_group.clear_pose_targets()
        print("clear outputs")
        variable = robot_b_group.get_current_pose()
        print (variable.pose)
        rospy.sleep(1)



###### Setup ########
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous =True)
scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()

robot_a_group = moveit_commander.MoveGroupCommander("robot_a")
robot_b_group = moveit_commander.MoveGroupCommander("robot_b")



display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)


robot_a_group.set_planning_time(10.0)
robot_b_group.set_planning_time(10.0)


#robot_b_group.set_named_target("robot_b_ready")
#plan = robot_b_group.go()
#rospy.sleep(2.0)

robot_a_group.set_named_target("robot_a_ready")
plan = robot_a_group.go()


pose_target = geometry_msgs.msg.Pose()

pose_target.orientation.w = 1.0
pose_target.position.x = 0.25
pose_target.position.y = -0.15
pose_target.position.z =0.2

robot_a_group.set_pose_target(pose_target)

print("Executing Move: Position3")
plan1 = robot_a_group.go(wait=True)
print("plan and execute")
robot_a_group.stop()
print("stop")
robot_a_group.clear_pose_targets()
print("clear outputs")
variable = robot_a_group.get_current_pose()
print (variable.pose)


pose_target2 = geometry_msgs.msg.Pose()

pose_target2.orientation.w = 1.0
pose_target2.position.x = 0.25
pose_target2.position.y = -0.25
pose_target2.position.z =0.35

robot_a_group.set_pose_target(pose_target2)

print("Executing Move: Position3")
plan1 = robot_a_group.go(wait=True)
print("plan and execute")
robot_a_group.stop()
print("stop")
robot_a_group.clear_pose_targets()
print("clear outputs")
variable = robot_a_group.get_current_pose()
print (variable.pose)



pose_target3 = geometry_msgs.msg.Pose()

pose_target3.orientation.w = 1.0
pose_target3.position.x = 0.35
pose_target3.position.y = 0.3
pose_target3.position.z =0.15

robot_b_group.set_pose_target(pose_target3)

print("Executing Move: Position3")
plan2 = robot_b_group.go(wait=True)

robot_b_group.stop()

robot_b_group.clear_pose_targets()

variable = robot_b_group.get_current_pose()
print (variable.pose)
rospy.sleep(1)


pose_target3 = geometry_msgs.msg.Pose()

pose_target3.orientation.w = 1.0
pose_target3.position.x = 0.35
pose_target3.position.y = 0.3
pose_target3.position.z =0.15

robot_b_group.set_pose_target(pose_target3)

print("Executing Move: Position3")
plan2 = robot_b_group.go(wait=True)
print("plan and execute")
robot_b_group.stop()
print("stop")
robot_b_group.clear_pose_targets()
print("clear outputs")
variable = robot_b_group.get_current_pose()
print (variable.pose)
rospy.sleep(1)


moveit_commander.roscpp_shutdown()