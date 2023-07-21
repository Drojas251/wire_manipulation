#!/usr/bin/env python3
from time import sleep
import rospy

# tf2 and Transformations
from tf.transformations import quaternion_from_euler
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool

# Camera capture
from cv_bridge import CvBridge,CvBridgeError
from collections import defaultdict

import cv2
import numpy as np 
import cam_calibration

import math

def transform_search_target(child_name: str, source: str, pos_adj, ori_adj) -> None:
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = source
    t.child_frame_id = "{}".format(child_name, source)

    t.transform.translation.x = ori_adj[0]
    t.transform.translation.y = ori_adj[1]
    t.transform.translation.z = ori_adj[2]
    
    q = quaternion_from_euler(pos_adj[0], pos_adj[1], pos_adj[2])

    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

def main():
    rospy.init_node('search_target')
    
    rate = rospy.Rate(60)

    # Define max and min for horizontal and vertical coordinates
    # +distance to panels, horizontal 0.325:-0.4, vertical -0.4:0.2
    MIN_X, MAX_X = -0.4, 0.3
    MIN_Y, MAX_Y = -0.4, 0.2
    dx, dy = 0.1, 0.1

    z, x, y, = 0.75, 0.2, -0.1 # initalize search target at middle origin position

    x_dir, y_dir = -1, 1 # direction to start the search
    x_min_reached, x_max_reached = x,x # both positions must be adjusted to change coordinate (e.g. .2, .2)
    y_min_reached, y_max_reached = y,y
    
    next_dir = 'x'
    while not rospy.is_shutdown():
        # if message received to move:
        move_flag = rospy.wait_for_message("move_flag", Bool)
        if move_flag.data:
            # decide if moving x or y
            # delta = global() RESUME HERE

            if next_dir == 'x':
                x_delta = x_dir * dx
                x_adj = x + x_delta
                if (x <= MAX_X and x >= MIN_X):
                    x = x_adj
                    
                if (x_dir > 0 and x > x_max_reached):
                    x_max_reached += dx
                    x_dir *= -1
                    next_dir = 'y'
                elif (x_dir < 0 and x < x_min_reached):
                    x_min_reached -= dx
                    x_dir *= -1
                    next_dir = 'y'

            elif next_dir == 'y':
                y_delta = y_dir * dy
                y_adj = y + y_delta
                if (y <= MAX_Y and y >= MIN_Y):
                    y = y_adj
                    
                if (y_dir > 0 and y > y_max_reached):
                    y_max_reached += dy
                    y_dir *= -1
                    next_dir = 'x'
                elif (y_dir < 0 and y < y_min_reached):
                    y_min_reached -= dy
                    y_dir *= -1
                    next_dir = 'x'

            sleep(1)

        transform_search_target("search_target", "camera_link", [0, 0, 0], [z, x, y])

        rate.sleep()


#*** Node Starts Here ***#
if __name__ == "__main__":
    main()
