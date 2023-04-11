#!/usr/bin/env python3

#ROS
import rospy
import geometry_msgs.msg
import sys
import moveit_commander
import math

from time import sleep
import tf2_ros

class SlipDetect:
    def __init__(self, slip_delta):
        # Distance in meters of how far cable end ArUco must move to quantify as slip
        self.slip_delta = slip_delta 
        # Robot services to get poses
        moveit_commander.roscpp_initialize(sys.argv)
        self.right_arm = moveit_commander.MoveGroupCommander("a_bot_arm")
        self.left_arm = moveit_commander.MoveGroupCommander("b_bot_arm")

    def monitor_dist(self, rate_input: float, end_grasping_arm: str):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rate = rospy.Rate(rate_input)
        while not rospy.is_shutdown():
            rate.sleep()
            try:
                end_pose = tfBuffer.lookup_transform("world", "aruco_0", rospy.Time())
                arm_pose = self.get_current_pose(end_grasping_arm)
                print(self.calc_dist(end_pose, arm_pose))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

    def calc_dist(self, end_pose, arm_pose):
        end_pose = end_pose.transform.translation
        arm_pose = arm_pose.pose.position
        return math.sqrt(math.pow(end_pose.x - arm_pose.x, 2) + math.pow(end_pose.y - arm_pose.y, 2) + math.pow(end_pose.z - arm_pose.z, 2))

    def get_current_pose(self, robot_id:str):
        return self.left_arm.get_current_pose() if robot_id == "left" else self.right_arm.get_current_pose()
        
def main():
    rospy.init_node('slip_detect', anonymous=True)

    slip_detector = SlipDetect(0)
    slip_detector.monitor_dist(10.0, "left")

    rospy.spin()

#*** Node Starts Here ***#
if __name__ == "__main__":
    main()
