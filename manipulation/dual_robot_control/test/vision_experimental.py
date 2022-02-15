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

# Robot Control 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import colorama
from colorama import Fore
import os.path

wire_thickness = 1
test_number = 3

save_path = '/usr/local/src/wire_manipulation_framework/src/wire_manipulation/testing/vision_testing/wire' + str(wire_thickness) + '/test' + str(test_number)
name_of_ground_truth_file = "vision_test_" + str(test_number) + "_ground_truth"  
name_of_sensed_file = "vision_test_" + str(test_number) + "_sensed_truth"

GroundTruth = os.path.join(save_path, name_of_ground_truth_file +".txt")
Sensed = os.path.join(save_path, name_of_sensed_file +".txt")


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

def robot_control(wire_grasping_robot,wire_grasp_pose,pull_vec,object_grasp_pose):
     rospy.wait_for_service('robot_control_service')
     try:
         robot_control_input = rospy.ServiceProxy('robot_control_service', GraspWire)
         response = robot_control_input(wire_grasping_robot,wire_grasp_pose,pull_vec,object_grasp_pose)
         return response
     except rospy.ServiceException as e:
         print("Service call failed: %s"%e)

def get_adjusted_points(type,raw_data):

    f_r = open(GroundTruth, 'r+')
    f_w = open(Sensed, 'w')
    s = f_r.readlines()
    adjusted_points = []

    if type == 'v':
        # compare with the z coordinate of the ends of ground truth data

        start = str(s[0]) # first end point 
        end = str(s[-1])

        start_compare = start.split(' ')[2]
        end_compare = end.split(' ')[2]

        start_compare = float(start_compare)
        end_compare = float(end_compare)

        end_points = []
        end_points.append(start_compare)
        end_points.append(end_compare)
        end_points.sort()

        
        for i in range(len(raw_data)):
            if (raw_data[i][2] >= end_points[0] and raw_data[i][2] <= end_points[1]):
                L = [str(raw_data[i][0]) + " ",str(raw_data[i][1])+ " ",str(raw_data[i][2]) + " ","\n"] 
                f_w.writelines(L)

    else:
        # compare with the y coordinate of the ends of ground truth data
        start = str(s[0]) # first end point 
        end = str(s[-1])

        start_compare = start.split(' ')[1]
        end_compare = end.split(' ')[1]

        start_compare = float(start_compare)
        end_compare = float(end_compare)

        print(start_compare)
        print(end_compare)

        end_points = []
        end_points.append(start_compare)
        end_points.append(end_compare)
        end_points.sort()
        print(end_points)
        print("")

        for i in range(len(raw_data)):
            if (raw_data[i][1] >= end_points[0] and raw_data[i][1] <= end_points[1]):
                L = [str(raw_data[i][0]) + " ",str(raw_data[i][1])+ " ",str(raw_data[i][2]) + " ","\n"] 
                f_w.writelines(L)
                
                #adjusted_points.append(raw_data[[i],:])



#*** Node Starts Here ***#
if __name__ == "__main__":
    rospy.init_node('listener', anonymous=True)
    topic = "/rscamera/depth/points"

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


    curve = WireGraspToolbox(raw_data)
    wire_ = curve.BCurve()

    get_adjusted_points('v', wire_)


        
        



        




    #rospy.spin()