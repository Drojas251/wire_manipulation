#!/usr/bin/env python3

import numpy as np
from wire_modeling.wire_sim import *
import rospy
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import PointCloud2
from wire_modeling_msgs.srv import *
from wire_modeling.wire_grasp_toolbox import WireGraspToolbox
import math

# Robot Control 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def get_wire_pointcloud(topic):
    points = rospy.wait_for_message(topic, PointCloud2)
    return points

def process_point_cloud_client(points):
     rospy.wait_for_service('process_point_cloud')
     try:
         wire_nodes = rospy.ServiceProxy('process_point_cloud', ProcessPointCloud)
         response = wire_nodes(points)
         return response
     except rospy.ServiceException as e:
         print("Service call failed: %s"%e)


#*** Node Starts Here ***#
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('listener', anonymous=True)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=20)
topic = "/rscamera/depth/points"
scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()

# Get segmented pointcloud data
print("STATUS: Getting PointCloud Instance")
points = get_wire_pointcloud(topic)

# Process PC data to get estimated nodes on wire
print("STATUS: Processing Pc with process_point_cloud_server")
wire_nodes = process_point_cloud_client(points) # output data structure is a posearray

# Convert PoseArray into Numpy Array
wire = np.zeros((3,20))
raw_data = np.zeros((len(wire_nodes.raw_points.poses),3))

for i in range(20):
    wire[[0],[i]] = wire_nodes.pose.poses[i].position.x
    wire[[1],[i]] = wire_nodes.pose.poses[i].position.y
    wire[[2],[i]] = wire_nodes.pose.poses[i].position.z


for i in range(len(wire_nodes.raw_points.poses)):
    raw_data[[i],[0]] = wire_nodes.raw_points.poses[i].position.x
    raw_data[[i],[1]] = wire_nodes.raw_points.poses[i].position.y
    raw_data[[i],[2]] = wire_nodes.raw_points.poses[i].position.z


# ***** insert wire info here *****
L = wire_nodes.wire_length # get length from point cloud process
M = 0.028
Ks = 13.1579
Kb = 6.13
Kd = 5.26
wire_model = WireModel(L,M,Ks,Kb,Kd) # create Wire object


#*** Define Grasp Object **** 
gop = np.array([[0.45],[0],[0.3]]) # position of grasp object
god = np.array([[0.05],[0.05],[0.05]]) # dimensions of grasp object -> model the grasp object as a box
# orientation of grasp object wrt to world in order (xyz)
rx = 0 # rot about x
ry = 0 # rot about y
rz = 3.14 # rot about z
grasp_object = GraspObject(gop,god,rx,ry,rz) # create grasp object, object 

# adding grasp object 
box_name = "grasp_object"

box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = "world"
box_pose.pose.orientation.w = 1.0
box_pose.pose.position.x = gop[0]  # above the panda_hand frame
box_pose.pose.position.y = gop[1]  # above the panda_hand frame
box_pose.pose.position.z = gop[2]  # above the panda_hand frame
scene.add_box(box_name, box_pose, size=(god[0], god[1], god[2]))

#*** Define the Configuration of the Dual Arm Setup ***
left_robot_location = np.array([[0],[0.1778],[0]])
right_robot_location = np.array([[0],[-0.1778],[0]])
robot_reach = 0.7
distance_apart = 0.3556
robots = DualRobotConfig(left_robot_location, right_robot_location, robot_reach, distance_apart) # dual robot object

#*** Initialize the WireSim Object ***
N = 20
wire_sim_ = WireSim(N,wire_model,grasp_object, robots )

#*** Call to WireSim class function to find pick and pull action **

wire2 = np.zeros((3,N))
wire2[0] = 0.5*np.ones((1,N))
wire2[1] = np.linspace(0,L,N) - 0.1
wire2[2] = 0.3*np.ones((1,N))

print("STATUS: Sending Wire Config to Simulator")
pick, pull , robot_to_grasp_wire = wire_sim_.simulate(wire)
print("")
print("UPDATE: Result Returned from Simulator")
print("")
print("pick ", pick.flatten())
print("pull ", pull.flatten())
print("")

if (len(pick) > 0 and len(pull) > 0):

    # Get grasp orientation as quaternion 
    grasp = WireGraspToolbox(raw_data)
    grasp_rotm, grasp_quat = grasp.get_wire_grasp_orientation(np.transpose(pick),np.transpose(pull))


    ###### Robot Control Setup ########
    # robot_b = left
    # robot_a = right

    if(robot_to_grasp_wire == "left"):
        robot_grasp_object = moveit_commander.MoveGroupCommander("robot_a")
        robot_grasp_wire = moveit_commander.MoveGroupCommander("robot_b")
    else:
        robot_grasp_object = moveit_commander.MoveGroupCommander("robot_b")
        robot_grasp_wire = moveit_commander.MoveGroupCommander("robot_a")

    robot_grasp_object.set_planning_time(10.0)
    robot_grasp_wire.set_planning_time(10.0)


    pose_target = geometry_msgs.msg.Pose()

    pose_target.orientation.w = grasp_quat[3]
    pose_target.orientation.x = grasp_quat[0]
    pose_target.orientation.y = grasp_quat[1]
    pose_target.orientation.z = grasp_quat[2]
    
    #pose_target.orientation.w = 1.0
    pose_target.position.x = float(pick[0]) - 0.20
    pose_target.position.y = float(pick[1])
    pose_target.position.z = float(pick[2])

    robot_grasp_wire.set_pose_target(pose_target)

    print("STATUS: Execute Robot Control")
    print("     executing action: grasping wire")
    plan1 = robot_grasp_wire.go(wait=True)
    robot_grasp_wire.stop()

    pull_distance = 0.06

    pose_target.position.x = pose_target.position.x + pull_distance*float(pull[0])
    pose_target.position.y = pose_target.position.y + pull_distance*float(pull[1])
    pose_target.position.z = pose_target.position.z + pull_distance*float(pull[2])

    pose_target.position.y = pose_target.position.y - 0.05

    robot_grasp_wire.set_pose_target(pose_target)
    print("     executing action: moving wire")
    plan1 = robot_grasp_wire.go(wait=True)
    robot_grasp_wire.stop()

    robot_grasp_wire.clear_pose_targets()



    grasp_target = geometry_msgs.msg.Pose()
    grasp_target.orientation.w = 1.0
    grasp_target.position.x = float(gop[0]) - 0.15
    grasp_target.position.y = float(gop[1])
    grasp_target.position.z = float(gop[2])

    robot_grasp_object.set_pose_target(grasp_target)

    print("     executing action: grasping object")
    plan1 = robot_grasp_object.go(wait=True)
    robot_grasp_object.stop()
    robot_grasp_object.clear_pose_targets()

    grasp_target.position.x = grasp_target.position.x + 0.08
    robot_grasp_object.set_pose_target(grasp_target)
    plan1 = robot_grasp_object.go(wait=True)
    robot_grasp_object.stop()
    robot_grasp_object.clear_pose_targets()


# remove box from scene
scene.remove_world_object(box_name)

moveit_commander.roscpp_shutdown()


#rospy.spin()