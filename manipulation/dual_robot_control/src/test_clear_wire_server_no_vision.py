#!/usr/bin/env python3

import numpy as np
from wire_modeling.wire_sim import *
import rospy
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import PointCloud2
from wire_modeling_msgs.srv import *
from dual_robot_msgs.srv import *
from wire_modeling.wire_grasp_toolbox import WireGraspToolbox
import math
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose

# Robot Control 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import colorama
from colorama import Fore

def get_wire_pointcloud(topic):
    points = rospy.wait_for_message(topic, PointCloud2)
    return points

def get_final_node_set(sorted_points,N):
    # N = number of nodes
    Wire = WireGraspToolbox(sorted_points)
    
    #curve_set = Bezier.Curve(t_points, sorted_points)
    curve_set = Wire.BCurve()

    final_node_set = np.zeros((N,3))

    for i in range(N):
        final_node_set[[i],:] = curve_set[[i*5],:]

    return final_node_set 

def get_wire_length(points):
    length = 0
    for i in range(len(points)-1):
        length = length + ((points[i,0] - points[i+1,0])**2 + (points[i,1] - points[i+1,1])**2 + (points[i,2] - points[i+1,2])**2 )**0.5

    return length

"""
def process_point_cloud_client(points):
     rospy.wait_for_service('process_point_cloud')
     try:
         wire_nodes = rospy.ServiceProxy('process_point_cloud', ProcessPointCloud)
         response = wire_nodes(points)
         return response
     except rospy.ServiceException as e:
         print("Service call failed: %s"%e)
"""

def robot_control(wire_grasping_robot,wire_grasp_pose,pull_vec,object_grasp_pose):
     rospy.wait_for_service('robot_control_service')
     try:
         robot_control_input = rospy.ServiceProxy('robot_control_service', GraspWire)
         response = robot_control_input(wire_grasping_robot,wire_grasp_pose,pull_vec,object_grasp_pose)
         return response
     except rospy.ServiceException as e:
         print("Service call failed: %s"%e)

#*** Node Starts Here ***#
if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('clear_it', anonymous=True)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=20)
    topic = "/rscamera/depth/points"
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()

    # Get segmented pointcloud data
    #print("STATUS: Getting PointCloud Instance")
    #points = get_wire_pointcloud(topic)

    # Process PC data to get estimated nodes on wire
    #print("STATUS: Processing Pc with process_point_cloud_server")
    #wire_nodes = process_point_cloud_client(points) 

    """
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
    """

    print("Creating Data")
    x = 0.39

    #raw_data = np.array([[x,-0.25,0.08],[x,-0.2,0.2],[x,-0.15,0.4],[x,-.05,0.48]]) # Wire1
    #raw_data = np.array([[x,-0.2,0.13],[x,-0.1,0.2],[x,0.0,0.2],[x,.1,0.15]]) # Wire2
    raw_data = np.array([[x,0,0.0],[x,0.02,0.15],[x,0.05,0.3],[x,0.03,0.45]]) # Wire3

    points = get_final_node_set(raw_data,20)
    wire_length = get_wire_length(points)

    wire = np.zeros((3,20))

    for i in range(20):
        wire[:,[i]] = np.transpose(points[[i],:])


    # visualize 
    pose_array = PoseArray()


    for i in range(20):
        pose = Pose()
        pose.position.x = points[i,0]
        pose.position.y = points[i,1]
        pose.position.z = points[i,2]
        pose.orientation.w = 1.0

        pose_array.poses.append(pose)  

    print("Got Wire Points")
    # ***** insert wire info here *****
    L = wire_length # get length from point cloud process
    M = 0.028
    Ks = 13.1579
    Kb = 6.13
    Kd = 5.26
    wire_model = WireModel(L,M,Ks,Kb,Kd) # create Wire object


    #*** Define Grasp Object **** 
    gop = np.array([[0.45],[0.0],[0.25]]) # position of grasp object
    god = np.array([[0.05],[0.05],[0.05]]) # dimensions of grasp object -> model the grasp object as a box
    # orientation of grasp object wrt to world in order (xyz)
    rx = 0 # rot about x
    ry = 0 # rot about y
    rz = 3.14 # rot about z
    grasp_object = GraspObject(gop,god,rx,ry,rz) # create grasp object, object 


    #*** Define the Configuration of the Dual Arm Setup ***
    left_robot_location = np.array([[0],[0.1778],[0]])
    right_robot_location = np.array([[0],[-0.1778],[0]])
    robot_reach = 0.7
    distance_apart = 0.3556
    robots = DualRobotConfig(left_robot_location, right_robot_location, robot_reach, distance_apart) # dual robot object

    #*** Initialize grasp ***#
    curve = WireGraspToolbox(raw_data)

    #*** Initialize the WireSim Object ***
    N = 20
    wire_sim_ = WireSim(N,wire_model,grasp_object, robots, curve )

    #*** Call to WireSim class function to find pick and pull action **


    print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Sending Wire Config to Simulator ")
    pick, pull , wire_grasping_robot = wire_sim_.simulate(wire)

    print(Fore.BLUE + "UPDATE:= " + Fore.WHITE + "Robot Actions Returned from Simulator ")
    print("     Pick Point ", pick.flatten())
    print("     Pull Vector ", pull.flatten())
    print("")


    
    if (len(pick) > 0 and len(pull) > 0):

        # Get grasp orientation as quaternion 
        grasp_rotm, grasp_quat = curve.get_wire_grasp_orientation(np.transpose(pick),np.transpose(pull))


        ###### Robot Control Setup ########
        # robot_b = left
        # robot_a = right
        print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Initiating Robots ")

        # Initializing the Wire Grasp Pose
        wire_grasp_pose = geometry_msgs.msg.Pose()
        wire_grasp_pose.orientation.w = grasp_quat[3]
        wire_grasp_pose.orientation.x = grasp_quat[0]
        wire_grasp_pose.orientation.y = grasp_quat[1]
        wire_grasp_pose.orientation.z = grasp_quat[2]
        wire_grasp_pose.position.x = float(pick[0]) 
        wire_grasp_pose.position.y = float(pick[1])
        wire_grasp_pose.position.z = float(pick[2])

        # Initializing Pull Vector
        pull_vec = geometry_msgs.msg.Vector3()
        pull_vec.x = float(pull[0])
        pull_vec.y = float(pull[1])
        pull_vec.z = float(pull[2])

        object_grasp_pose = geometry_msgs.msg.Pose()
        object_grasp_pose.orientation.w = 1.0
        object_grasp_pose.position.x = float(gop[0]) 
        object_grasp_pose.position.y = float(gop[1])
        object_grasp_pose.position.z = float(gop[2])

        status = robot_control(wire_grasping_robot,wire_grasp_pose,pull_vec,object_grasp_pose)

        if status == True:
            print("SUCCESS")
        else:
            print("FAIL")

        




