#!/usr/bin/env python3

import numpy as np
from wire_modeling.wire_sim import *
import rospy
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import PointCloud2
from wire_modeling_msgs.srv import *
from wire_modeling.wire_grasp_toolbox import WireGraspToolbox
import math
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R

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

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('clear_it', anonymous=True)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=20)
    topic = "/rscamera/depth/points"
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    rate = rospy.Rate(10) # 10hz

    try:

        pre_grasp_offset = 0.05
        post_grasp_offset = 0.04

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
        #raw_data = np.array([[x,-0.25,0.08],[x,-0.2,0.2],[x,-0.15,0.4],[x,-.05,0.48]])
        raw_data = np.array([[x,-0.2,0.33],[x,-0.1,0.4],[x,0.0,0.4],[x,.1,0.35]]) # Wire2
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

        print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Got Wire Config from Vision ")
        rospy.sleep(1)
        # ***** insert wire info here *****
        L = wire_length # get length from point cloud process
        M = 0.028
        Ks = 13.1579
        Kb = 6.13
        Kd = 5.26
        wire_model = WireModel(L,M,Ks,Kb,Kd) # create Wire object


        #*** Define Grasp Object **** 
        gop = np.array([[0.42],[0.0],[0.35]]) # position of grasp object
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

        #*** Initialize grasp ***#
        curve = WireGraspToolbox(raw_data)

        #*** Initialize the WireSim Object ***
        N = 20
        wire_sim_ = WireSim(N,wire_model,grasp_object, robots, curve )

        #*** Call to WireSim class function to find pick and pull action **


        print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Sending Wire Config to Simulator ")
        pick, pull , robot_to_grasp_wire = wire_sim_.simulate(wire)

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

            if(robot_to_grasp_wire == "left"):
                robot_grasp_object = moveit_commander.MoveGroupCommander("robot_a")
                robot_grasp_wire = moveit_commander.MoveGroupCommander("robot_b")
                print(Fore.BLUE + "UPDATE:= " + Fore.WHITE + "Assigning Robot Tasks")
                print(Fore.YELLOW + "     Left Robot is Grasping Wire")
                print(Fore.YELLOW + "     Right Robot is Grasping Object")
            else:
                robot_grasp_object = moveit_commander.MoveGroupCommander("robot_b")
                robot_grasp_wire = moveit_commander.MoveGroupCommander("robot_a")
                print(Fore.BLUE + "UPDATE:= " + Fore.WHITE + "Assigning Robot Tasks")
                print(Fore.YELLOW + "     Right Robot is Grasping Wire")
                print(Fore.YELLOW + "     Left Robot is Grasping Object")

            robot_grasp_object.set_planning_time(10.0)
            robot_grasp_wire.set_planning_time(10.0)


            # Move to the Computed Grasp Pose to grasp the wire 
            pose_target = geometry_msgs.msg.Pose()

            pose_target.orientation.w = grasp_quat[3]
            pose_target.orientation.x = grasp_quat[0]
            pose_target.orientation.y = grasp_quat[1]
            pose_target.orientation.z = grasp_quat[2]

            q = pose_target.orientation
            grasp_obj_rotm = R.from_quat([q.x,q.y,q.z,q.w])
            grasp_obj_rotm = grasp_obj_rotm.as_matrix()
            pre_grasp = pre_grasp_offset*grasp_obj_rotm[:,[0]]

            pose_target.position.x = float(pick[0]) - float(pre_grasp[0])
            pose_target.position.y = float(pick[1]) - float(pre_grasp[1])
            pose_target.position.z = float(pick[2]) - float(pre_grasp[2])

            robot_grasp_wire.set_pose_target(pose_target)

            print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "Execute Robot Control")
            print("     Executing action: pre-grasp of wire")
            rate.sleep()
            plan1 = robot_grasp_wire.go(wait=True)
            robot_grasp_wire.stop()


            # Going in for Grasps
            

            pose_target.position.x = float(pick[0]) 
            pose_target.position.y = float(pick[1]) 
            pose_target.position.z = float(pick[2]) 

            robot_grasp_wire.set_pose_target(pose_target)
            print("     Executing action: grasping wire")
            plan1 = robot_grasp_wire.go(wait=True)
            rate.sleep()
            robot_grasp_wire.stop()


            # Cartesian move to move wire in the direction of the pull vector
            pull_distance = 0.12
            waypoints = []

            wpose = robot_grasp_wire.get_current_pose().pose
            wpose.position.x += pull_distance*float(pull[0])  
            wpose.position.y += pull_distance*float(pull[1])  
            wpose.position.z += pull_distance*float(pull[2])
            waypoints.append(copy.deepcopy(wpose))

            (plan1, fraction) = robot_grasp_wire.compute_cartesian_path(waypoints, 0.01, 0.0)  

            print("     Executing action: moving wire")
            rate.sleep()
            robot_grasp_wire.execute(plan1, wait=True)


            ## Setting up Grasps for pick and place
            grasps = [] 

            grasp1 = geometry_msgs.msg.Quaternion()
            grasp2 = geometry_msgs.msg.Quaternion()
            grasp3 = geometry_msgs.msg.Quaternion()
            grasp4 = geometry_msgs.msg.Quaternion()
            grasp5 = geometry_msgs.msg.Quaternion()
            grasp6 = geometry_msgs.msg.Quaternion()
            grasp7 = geometry_msgs.msg.Quaternion()
            grasp8 = geometry_msgs.msg.Quaternion()
            grasp9 = geometry_msgs.msg.Quaternion()
            grasp10 = geometry_msgs.msg.Quaternion()


            # Grasp Obj forward
            grasp1.w = 1.0
            grasps.append(grasp1)

            # Grasp obj on right option 1
            grasp7.x = 0.7071068
            grasp7.y = 0.0
            grasp7.z = 0.0
            grasp7.w = 0.7071068
            grasps.append(grasp7)

            # Grasp obj on right option 1
            grasp8.x = -0.7071068
            grasp8.y = 0.0
            grasp8.z = 0.0
            grasp8.w = 0.7071068
            grasps.append(grasp8)

            # Grasp obj on right option 1
            grasp9.x = 0.0
            grasp9.y = 0.0
            grasp9.z = 0.258819
            grasp9.w = 0.9659258
            grasps.append(grasp9)

            # Grasp obj on right option 1
            grasp10.x = 0.0
            grasp10.y = 0.0
            grasp10.z = 0.258819
            grasp10.w = 0.9659258
            grasps.append(grasp10)

            # Grasp obj on right option 1
            grasp2.x = -0.5
            grasp2.y = -0.5
            grasp2.z = 0.5
            grasp2.w = 0.5
            grasps.append(grasp2)

            # Grasp obj on left option 1
            grasp3.x = 0.5
            grasp3.y = -0.5
            grasp3.z = -0.5
            grasp3.w = 0.5
            grasps.append(grasp3)

            # Grasp obj on bottom 
            grasp4.x = 0
            grasp4.y = -0.7071068
            grasp4.z = 0
            grasp4.w = 0.7071068
            grasps.append(grasp4)

            # Grasp obj on left option 2
            grasp5.x = -0.5
            grasp5.y = 0.5
            grasp5.z = -0.5
            grasp5.w = 0.5
            grasps.append(grasp5)

            # Grasp obj on right option 1
            grasp6.x = 0.5
            grasp6.y = 0.5
            grasp6.z = 0.5
            grasp6.w = 0.5
            grasps.append(grasp6)

            pre_grasp_offset = 0.05
            post_grasp_offset = 0.04


            for i in range(len(grasps)):
                grasp_target = geometry_msgs.msg.Pose()
                grasp_target.orientation = grasps[i]

                q = grasp_target.orientation
                grasp_obj_rotm = R.from_quat([q.x,q.y,q.z,q.w])
                grasp_obj_rotm = grasp_obj_rotm.as_matrix()

                pre_grasp = pre_grasp_offset*grasp_obj_rotm[:,[0]]

                grasp_target.position.x = float(gop[0]) - float(pre_grasp[0])
                grasp_target.position.y = float(gop[1]) - float(pre_grasp[1])
                grasp_target.position.z = float(gop[2]) - float(pre_grasp[2])

                print("     Checking Grasp Orientaiton #" + str(i+1))

                robot_grasp_object.set_pose_target(grasp_target)
                error_code_val, plan1, planning_time, error_code = robot_grasp_object.plan()
                rate.sleep()

                success = (error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)
                
                if success == True:
                    print(Fore.GREEN + "     Valid Grasp Found")
                    print(grasp_target.orientation)

                    print(Fore.WHITE + "     Executing action: grasping object")
                    robot_grasp_object.execute(plan1)
                    robot_grasp_object.clear_pose_targets()

                    # Going in for Grasps
                    #post_grasp = post_grasp_offset*grasp_obj_rotm[:,[0]]

                    grasp_target.position.x = float(gop[0]) 
                    grasp_target.position.y = float(gop[1]) 
                    grasp_target.position.z = float(gop[2])

                    robot_grasp_object.set_pose_target(grasp_target)
                    plan1 = robot_grasp_object.go(wait=True)
                    robot_grasp_object.stop()
                    robot_grasp_object.clear_pose_targets()

                    break
                else:
                    robot_grasp_object.clear_pose_targets()
                    if i >= len(grasps):
                        print(Fore.RED + "NO SOLUTION FOUND")

        # remove box from scene
        scene.remove_world_object(box_name)
        moveit_commander.roscpp_shutdown()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return



#*** Node Starts Here ***#
if __name__ == "__main__":
    main()


