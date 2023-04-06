#!/usr/bin/env python3

#ROS
import rospy
import geometry_msgs.msg

# from dual_robot_msgs.srv import *
from time import sleep
import tf2_ros

class SlipDetect:
    def __init__(self, slip_delta):
        # Distance in meters of how far cable end ArUco must move to quantify as slip
        self.slip_delta = slip_delta 

    def get_transforms(self, rate_input: float, wire_grasping_arm: str):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        arm = "a_bot_base_link" if wire_grasping_arm == "left" else "b_bot_base_link"
        rate = rospy.Rate(rate_input)
        while not rospy.is_shutdown():
            rate.sleep()
            try:
                end_pose = tfBuffer.lookup_transform("world", "aruco_0", rospy.Time())
                arm_pose = tfBuffer.lookup_transform("world", arm, rospy.Time())
                # print(end_pose)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # print("error")
                continue

    def monitor_aruco(self, rate_input):
        self.get_transforms(rate_input, "left")
        

def main():
    rospy.init_node('slip_detect', anonymous=True)

    slip_detector = SlipDetect(0)
    slip_detector.monitor_aruco(10.0)

    rospy.spin()

#*** Node Starts Here ***#
if __name__ == "__main__":
    main()
