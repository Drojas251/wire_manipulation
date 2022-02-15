#!/usr/bin/env python3

import numpy as np
from wire_modeling.wire_sim import *
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from wire_modeling.wire_grasp_toolbox import WireGraspToolbox, rotm
import math
import os.path

wire_thickness = 1
test_number = 3

save_path = '/usr/local/src/wire_manipulation_framework/src/wire_manipulation/testing/vision_testing/wire' + str(wire_thickness) + '/test' + str(test_number)
name_of_ground_truth_file = "vision_test_" + str(test_number) + "_ground_truth"  
name_of_sensed_file = "vision_test_" + str(test_number) + "_sensed_truth"

GroundTruth = os.path.join(save_path, name_of_ground_truth_file +".txt")

def sort_tags(tags):
    sorted_tags = []
    size = len(tags.detections)

    for i in range(size):
        for j in range(size):
            if tags.detections[j].id[0] == i:
                sorted_tags.append(tags.detections[j])

    return sorted_tags

def get_tags(topic):
    tags = rospy.wait_for_message(topic, AprilTagDetectionArray)
    size = len(tags.detections)
    points = np.zeros((size,3))

    new_tags = sort_tags(tags)

    angle = math.pi/2
    ROT = rotm(-angle,angle,0)

    
    f = open(GroundTruth, 'w')

    for i in range(size):
        #points[[i],[0]] = tags.detections[i].pose.pose.pose.position.x
        #points[[i],[1]] = tags.detections[i].pose.pose.pose.position.y
        #points[[i],[2]] = tags.detections[i].pose.pose.pose.position.z

        points[[i],[0]] = new_tags[i].pose.pose.pose.position.x
        points[[i],[1]] = new_tags[i].pose.pose.pose.position.y
        points[[i],[2]] = new_tags[i].pose.pose.pose.position.z

        points[[i],:] = np.transpose(ROT@np.transpose(points[[i],:])) + np.array((-0.3556,0.015,0.4064))  

        L = [str(points[i][0]) + " ",str(points[i][1])+ " ",str(points[i][2]) + " ","\n"] 
        f.writelines(L)
  
    f.close()
    return points



#*** Node Starts Here ***#
if __name__ == "__main__":
    rospy.init_node('listener', anonymous=True)
    april_tags_topic = "/tag_detections"
    points = get_tags(april_tags_topic)






