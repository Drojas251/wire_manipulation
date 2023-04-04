#!/usr/bin/env python3

#ROS
import rospy
from sensor_msgs.msg import PointCloud2
import geometry_msgs.msg

# custom libs
from wire_modeling.wire_sim import Collisions,TargetObject,WireModel,WireSim
from wire_modeling_msgs.srv import *
# from dual_robot_msgs.srv import *
from wire_modeling.wire_grasp_toolbox import WireGraspToolbox, rotm
from time import sleep
import tf2_ros

from robot_services import RobotControl

#python
import numpy as np
import math
from colorama import Fore

# returns a pointcloud instance
def get_wire_pointcloud(topic):
    points = rospy.wait_for_message(topic, PointCloud2)
    return points

# Client to call to get wire model 
def process_point_cloud_client(points):
     rospy.wait_for_service('process_point_cloud')
     try:
         wire_nodes = rospy.ServiceProxy('process_point_cloud', ProcessPointCloud)
         response = wire_nodes(points)
         return response
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
    topic = "/rscamera/depth/points"

    robot_control = RobotControl()

    wire_grasping_robot = "left"
    object_grasping_robot = "right"
    status = robot_control.move_to_target(wire_grasping_robot, 'sleep')
    status = robot_control.move_to_target(object_grasping_robot, 'sleep')

    # Open grippers on both arms
    status = robot_control.set_gripper(wire_grasping_robot, "open")
    status = robot_control.set_gripper(object_grasping_robot, "open")

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
    #Ks = 237.5
    #Kb = 25
    #Kd = 125
    wire_model = WireModel(L,M,Ks,Kb,Kd,wire_nodes.wire_class) # create Wire object


    #*** Define Grasp Object **** 
    gop = np.array([[0.48],[-0.03],[0.27]]) # position of grasp object
    god = np.array([[0.05],[0.05],[0.05]]) # dimensions of grasp object -> model the grasp object as a box
    # orientation of grasp object wrt to world in order (xyz)
    rx = 0 # rot about x
    ry = 0 # rot about y
    rz = 3.14 # rot about z
    target_object = TargetObject(gop,god,rx,ry,rz) # create grasp object, object 

    #*** Get bezier curve ***#
    curve = WireGraspToolbox(raw_data)

    #*** Collision Object ***#
    collisions = Collisions()
    collisions.make_box_env_collision_obj(0.05,0.76,0.6,np.array([0.53,0,0.3])) # Front panel
    collisions.make_box_env_collision_obj(0.457,0.05,0.6,np.array([0.3,-0.382,0.3])) # Side Panel
    collisions.make_box_env_collision_obj(0.05,0.05,0.05,np.array([0.48,0.03,0.27])) # Target Object

    #*** Initialize the WireSim Object ***
    N = 20
    wire_sim_ = WireSim(N,wire_model,target_object, curve, collisions )

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

        # Set the Wire Grasp Pose
        wire_grasp_pose = geometry_msgs.msg.Pose()
        wire_grasp_pose.orientation.w = grasp_quat[3]
        wire_grasp_pose.orientation.x = grasp_quat[0]
        wire_grasp_pose.orientation.y = grasp_quat[1]
        wire_grasp_pose.orientation.z = grasp_quat[2]
        wire_grasp_pose.position.x = float(pick[0]) 
        wire_grasp_pose.position.y = float(pick[1])
        wire_grasp_pose.position.z = float(pick[2])

        # Set Pull Vector
        pull_vec = geometry_msgs.msg.Vector3()
        pull_vec.x = float(pull[0])
        pull_vec.y = float(pull[1])
        pull_vec.z = float(pull[2])

        # Set the target pose
        object_grasp_pose = geometry_msgs.msg.Pose()
        object_grasp_pose.orientation.w = 1.0
        object_grasp_pose.position.x = float(gop[0]) 
        object_grasp_pose.position.y = float(gop[1])
        object_grasp_pose.position.z = float(gop[2])

        #Task Executor
        if(wire_grasping_robot == "left"):
            object_grasping_robot = "right"
        else:
            object_grasping_robot = "left"

        # BEGIN ROUTINE
        # Begin both arms in sleep
        status = robot_control.move_to_target(wire_grasping_robot, 'sleep')
        status = robot_control.move_to_target(object_grasping_robot, 'sleep')

        # Set both arms to a ready state
        status = robot_control.move_to_target(wire_grasping_robot, 'ready')
        status = robot_control.move_to_target(object_grasping_robot, 'ready')
        # Open grippers on both arms
        status = robot_control.set_gripper(wire_grasping_robot, "open")
        status = robot_control.set_gripper(object_grasping_robot, "open")

        # Send object grasp robot to pre-grasp, encountering wire
        joint_goal = [62, -30, 57, 77, -64, -59]
        joint_goal = [x * np.pi / 180 for x in joint_goal]
        status = robot_control.move_to_joint_goal(object_grasping_robot, joint_goal)
        sleep(3)
        status = robot_control.move_to_target(object_grasping_robot, 'ready')

        #grasp wire
        status = robot_control.grasp_wire(wire_grasping_robot,wire_grasp_pose,pull_vec)
        status = robot_control.set_gripper(wire_grasping_robot, "close")

        #grasp target
        status = robot_control.grasp_object(object_grasping_robot,object_grasp_pose)
        status = robot_control.set_gripper(object_grasping_robot, "close")
        
        # sleep right arm after grasping object
        status = robot_control.move_to_target(object_grasping_robot, 'ready')

        # sleep left arm after brushing wire
        status = robot_control.set_gripper(wire_grasping_robot, "open")
        status = robot_control.move_to_target(wire_grasping_robot, 'ready')
        
    #rospy.spin()
    