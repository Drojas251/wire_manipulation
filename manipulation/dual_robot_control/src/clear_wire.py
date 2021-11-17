#!/usr/bin/env python3

import numpy as np
from wire_modeling.wire_sim import *
import rospy
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import PointCloud2
from wire_modeling_msgs.srv import *

def get_wire_pointcloud(topic):
    points = rospy.wait_for_message(topic, PointCloud2)
    return points

def process_point_cloud_client(points):
     rospy.wait_for_service('process_point_cloud')
     try:
         wire_nodes = rospy.ServiceProxy('process_point_cloud', ProcessPointCloud)
         response = wire_nodes(points)
         return response.pose
     except rospy.ServiceException as e:
         print("Service call failed: %s"%e)


#*** Node Starts Here ***#
rospy.init_node('listener', anonymous=True)
topic = "/rscamera/depth/points"

# Get segmented pointcloud data
points = get_wire_pointcloud(topic)

# Process PC data to get estimated nodes on wire
wire_nodes = process_point_cloud_client(points) # output data structure is a posearray

# Convert PoseArray into Numpy Array
wire = np.zeros((3,20))

for i in range(20):
    wire[[0],[i]] = wire_nodes.poses[i].position.x
    wire[[1],[i]] = wire_nodes.poses[i].position.y
    wire[[2],[i]] = wire_nodes.poses[i].position.z


# ***** insert wire info here *****
L = 0.35
M = 0.028
Ks = 13.1579
Kb = 6.13
Kd = 5.26
wire_model = WireModel(L,M,Ks,Kb,Kd) # create Wire object


#*** Define Grasp Object **** 
gop = np.array([[0.55],[0.05],[0.3]]) # position of grasp object
god = np.array([[0.15],[0.15],[0.15]]) # dimensions of grasp object -> model the grasp object as a box
# orientation of grasp object wrt to world in order (xyz)
rx = 0 # rot about x
ry = 0 # rot about y
rz = 3.14 # rot about z
grasp_object = GraspObject(gop,god,rx,ry,rz) # create grasp object, object 


#*** Define the Configuration of the Dual Arm Setup ***
left_robot_location = np.array([[0],[0.15],[0]])
right_robot_location = np.array([[0],[-0.15],[0]])
robot_reach = 0.7
distance_apart = 0.3
robots = DualRobotConfig(left_robot_location, right_robot_location, robot_reach, distance_apart) # dual robot object

#*** Initialize the WireSim Object ***
N = 20
wire_sim_ = WireSim(N,wire_model,grasp_object, robots )

#*** Call to WireSim class function to find pick and pull action **
wire = np.zeros((3,N))
wire[0] = 0.5*np.ones((1,N))
wire[1] = np.linspace(0,L,N) - 0.1
wire[2] = 0.3*np.ones((1,N))

robot_actions = wire_sim_.simulate(wire)
print(robot_actions)


rospy.spin()