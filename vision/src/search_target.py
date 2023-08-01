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
import math
import numpy as np 
import cam_calibration

import math

NODE_OFFSETS = { # Specified z, x, y
    0 : [0, 0, 1],
    1 : [0, 1, 1],
    2 : [0, 1, 0],
    3 : [0, 1, -1],
    4 : [0, 0, -1],
    5 : [0, -1, -1],
    6 : [0, -1, 0],
    7 : [0, -1, 1],
}

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

    ### Variables for spiral positioning
    # Define max and min for horizontal and vertical coordinates
    # +distance to panels, horizontal 0.325:-0.4, vertical -0.4:0.2
    MIN_X, MAX_X = -0.4, 0.3
    MIN_Y, MAX_Y = -0.4, 0.2
    dx, dy = 0.1, 0.1

    z_pos, x_pos, y_pos, = 0.75, 0.2, -0.1 # initalize search target at middle origin position; updated through search routine
    z_ori, x_ori, y_ori, = 0,0,0 # initialize search target orientation; updated through search at a given target frame pose

    x_dir, y_dir = -1, 1 # direction to start the search
    x_min_reached, x_max_reached = x_pos,x_pos # both positions must be adjusted to change coordinate (e.g. .2, .2)
    y_min_reached, y_max_reached = y_pos,y_pos

    POS_SAVE = {
        'z' : z_pos,
        'x' : x_pos,
        'y' : y_pos,
    }

    next_dir = 'x'
    while not rospy.is_shutdown(): # Outer loop controls moving to each spiral node
        try:
            # if message received to move position, adjust search target's x,y positions
            move_flag_pos = rospy.wait_for_message("move_flag_pos", Bool, 2.5)
            if move_flag_pos.data:
                # Load saved spiral node position and save for next iteration
                z_pos, x_pos, y_pos = POS_SAVE['z'], POS_SAVE['x'], POS_SAVE['y']
                POS_SAVE['z'], POS_SAVE['x'], POS_SAVE['y'] = z_pos, x_pos, y_pos
                
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
        except rospy.exceptions.ROSException:
            try:
                move_flag_ori = rospy.wait_for_message("move_flag_ori", Bool, 2.5) # wait for signal to start the search at node
                print(move_flag_ori)
                if move_flag_ori.data:
                    print("hit move_flag_ori")
                    node_variation_counter = 0
                    while node_variation_counter < len(NODE_OFFSETS):
                        try:
                            # async issue? messages arent hitting in time or something
                            move_flag_ori2 = rospy.wait_for_message("move_flag_ori2", Bool, 2.5) # move the search target to each search position
                            if move_flag_ori2.data:
                                print("hit move_flag_ori2")
                                # adjust pos and ori for sub 
                                x_pos = POS_SAVE['x'] + (NODE_OFFSETS[node_variation_counter][1]*dx/2)
                                y_pos = POS_SAVE['y'] + (NODE_OFFSETS[node_variation_counter][2]*dy/2)
                                node_variation_counter += 1
                        except rospy.exceptions.ROSException:
                            pass
                        finally:
                            print("move sub")
                            transform_search_target("search_target", "camera_link", [z_ori, x_ori, y_ori], [z_pos, x_pos, y_pos])
                sleep(1)
            except rospy.exceptions.ROSException:
                pass
        finally:
            print("move main")
            transform_search_target("search_target", "camera_link", [z_ori, x_ori, y_ori], [z_pos, x_pos, y_pos])

        rate.sleep()


#*** Node Starts Here ***#
if __name__ == "__main__":
    main()
