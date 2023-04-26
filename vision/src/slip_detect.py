#!/usr/bin/env python3

#ROS
import rospy
import geometry_msgs.msg
from std_msgs.msg import Bool
import sys
import moveit_commander
import math

from time import sleep
import tf2_ros

import time

class SlipDetect:
    def __init__(self):
        # Distance in meters of how far cable end ArUco must move to quantify as slip
        # Robot services to get poses
        moveit_commander.roscpp_initialize(sys.argv)

        tracking_arm = rospy.get_name().strip('/').split('_')[0]
        self.tracking_arm = "a_bot_arm" if tracking_arm == "right" else "b_bot_arm"
        self.tracking_arm_command = moveit_commander.MoveGroupCommander(self.tracking_arm)
        self.end_pose = {}

        # Publisher for delta surpassed
        self.marker_delta_flag_pub = rospy.Publisher("/{}_marker_delta_flag".format(self.tracking_arm), Bool, queue_size=1)

    def monitor_dist(self, rate_input: float, slip_delta: float):
        """
        Parameters:
            rate_input (float): hertz for rate to check Euclidean distance between marker and executing arm
            end_grasping_arm (str): string specifying whether left or right arm attempted at wire and slipped from
            slip_delta (float): distance in meters of how far marker and arm must be to quantify flag raise
        """
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rate = rospy.Rate(rate_input)

        while not rospy.is_shutdown(): # loop takes ~0.099 - 0.1 seconds; 99.5ms
            try:
                self.end_pose = tfBuffer.lookup_transform("world", "aruco_0", rospy.Time())
                arm_pose = self.get_current_pose(self.tracking_arm)
                euclidean_dist = self.calc_dist(self.end_pose, arm_pose)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            
            marker_delta_flag = euclidean_dist > slip_delta
            print("\n---")
            print("ArUco Pose =\nx: {}\ny: {}\nz: {}".format(self.end_pose.transform.translation.x, self.end_pose.transform.translation.y, self.end_pose.transform.translation.z))
            print("\n{} Arm Pose =\nx: {}\ny: {}\nz: {}".format(self.tracking_arm, arm_pose.pose.position.x, arm_pose.pose.position.y, arm_pose.pose.position.z))
            print("\nEuclidean Distance =\n{}\n".format(self.calc_dist(self.end_pose, arm_pose)))
            if marker_delta_flag:
                print("\n********************\nDISTANCE SURPASSED =\nDistance Limit: {}\nLive distance: {}\n".format(slip_delta, euclidean_dist))
            
            self.marker_delta_flag_pub.publish(marker_delta_flag)
            rate.sleep()

    def calc_dist(self, end_pose, arm_pose):
        end_pose = end_pose.transform.translation
        arm_pose = arm_pose.pose.position
        return math.sqrt(math.pow(end_pose.x - arm_pose.x, 2) + math.pow(end_pose.y - arm_pose.y, 2) + math.pow(end_pose.z - arm_pose.z, 2))

    def get_current_pose(self, robot_id:str):
        return self.tracking_arm_command.get_current_pose()
        
def main():
    rospy.init_node('slip_detect', anonymous=True) #not getting param right here

    
    slip_detector = SlipDetect()
    slip_detector.monitor_dist(0.4, .20) # 0.4hz ~ 2.5 seconds, .20 meter slip delta

    rospy.spin()

#*** Node Starts Here ***#
if __name__ == "__main__":
    main()
