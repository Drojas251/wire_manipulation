#!/usr/bin/env python3

# import rospy

# # tf2 and Transformations
# import tf2_ros
# from math import pi
# from time import sleep
# from tf.transformations import quaternion_from_euler
# from geometry_msgs.msg import TransformStamped
# from std_msgs.msg import Bool

# NODE_OFFSETS = { # Specified z, x, y
#     0 : [0, 0, 1],
#     1 : [0, 1, 1],
#     2 : [0, 1, 0],
#     3 : [0, 1, -1],
#     4 : [0, 0, -1],
#     5 : [0, -1, -1],
#     6 : [0, -1, 0],
#     7 : [0, -1, 1],
# }

# # Definitions for angles constant across all subnodes
# X_ANG = pi/6
# Y_ANG = pi/6
# NODE_ORI_OFFSETS = { # Specified z, x, y
#     0 : [0, 0, 0],
#     1 : [0, X_ANG, 0],
#     2 : [0, X_ANG, -Y_ANG],
#     3 : [0, 0, -Y_ANG],
#     4 : [0, -X_ANG, -Y_ANG],
#     5 : [0, -X_ANG, 0],
#     6 : [0, -X_ANG, Y_ANG],
#     7 : [0, 0, Y_ANG],
#     8 : [0, X_ANG, Y_ANG],
# }

# def transform_search_target(child_name: str, source: str, pos_adj, ori_adj) -> None:
#     br = tf2_ros.TransformBroadcaster()
#     t = TransformStamped()

#     t.header.stamp = rospy.Time.now()
#     t.header.frame_id = source
#     t.child_frame_id = "{}".format(child_name, source)

#     t.transform.translation.x = ori_adj[0]
#     t.transform.translation.y = ori_adj[1]
#     t.transform.translation.z = ori_adj[2]
    
#     q = quaternion_from_euler(pos_adj[0], pos_adj[1], pos_adj[2])

#     t.transform.rotation.x = q[0]
#     t.transform.rotation.y = q[1]
#     t.transform.rotation.z = q[2]
#     t.transform.rotation.w = q[3]

#     br.sendTransform(t)

# def main():
#     rospy.init_node('search_target')
    
#     rate = rospy.Rate(60)

#     ### Variables for spiral positioning
#     # Define max and min for horizontal and vertical coordinates
#     # +distance to panels, horizontal 0.325:-0.4, vertical -0.4:0.2
#     MIN_X, MAX_X = -0.4, 0.3
#     MIN_Y, MAX_Y = -0.4, 0.2
#     dx, dy = 0.1, 0.1

#     z_pos, x_pos, y_pos, = 0.7, 0.2, -0.1 # initalize search target at middle origin position; updated through search routine
#     z_ori, x_ori, y_ori, = 0,0,0 # initialize search target orientation; updated through search at a given target frame pose

#     x_dir, y_dir = -1, 1 # direction to start the search
#     x_min_reached, x_max_reached = x_pos,x_pos # both positions must be adjusted to change coordinate (e.g. .2, .2)
#     y_min_reached, y_max_reached = y_pos,y_pos

#     POS_SAVE = {
#         'z' : z_pos,
#         'x' : x_pos,
#         'y' : y_pos,
#     }

#     next_dir = 'x'
#     while not rospy.is_shutdown(): # Outer loop controls moving to each spiral node
#         try:
#             # if message received to move position, adjust search target's x,y positions
#             move_flag_pos = rospy.wait_for_message("move_flag_pos", Bool, 2.5)
#             if move_flag_pos.data:
#                 # Load saved spiral node position and save for next iteration
#                 z_pos, x_pos, y_pos = POS_SAVE['z'], POS_SAVE['x'], POS_SAVE['y']
#                 POS_SAVE['z'], POS_SAVE['x'], POS_SAVE['y'] = z_pos, x_pos, y_pos
                
#                 if next_dir == 'x':
#                     x_delta = x_dir * dx
#                     x_adj = x + x_delta
#                     if (x <= MAX_X and x >= MIN_X):
#                         x = x_adj
                        
#                     if (x_dir > 0 and x > x_max_reached):
#                         x_max_reached += dx
#                         x_dir *= -1
#                         next_dir = 'y'
#                     elif (x_dir < 0 and x < x_min_reached):
#                         x_min_reached -= dx
#                         x_dir *= -1
#                         next_dir = 'y'

#                 elif next_dir == 'y':
#                     y_delta = y_dir * dy
#                     y_adj = y + y_delta
#                     if (y <= MAX_Y and y >= MIN_Y):
#                         y = y_adj
                        
#                     if (y_dir > 0 and y > y_max_reached):
#                         y_max_reached += dy
#                         y_dir *= -1
#                         next_dir = 'x'
#                     elif (y_dir < 0 and y < y_min_reached):
#                         y_min_reached -= dy
#                         y_dir *= -1
#                         next_dir = 'x'
#             sleep(1)
#         except rospy.exceptions.ROSException:
#             node_variation_counter = 0
#             while node_variation_counter < len(NODE_OFFSETS)-1:
#                 except_flag = False
#                 try:
#                     move_flag_ori2 = rospy.wait_for_message("move_flag_ori", Bool, 2.5) # move the search target to each search position
#                     if move_flag_ori2.data:
#                         print("hit move_flag_ori")
#                         # adjust pos and ori for sub 
#                         x_pos = POS_SAVE['x'] + (NODE_OFFSETS[node_variation_counter][1]*dx/2)
#                         y_pos = POS_SAVE['y'] + (NODE_OFFSETS[node_variation_counter][2]*dy/2)
#                         node_variation_counter += 1
#                 except rospy.exceptions.ROSException:
#                     except_flag = True
#                 finally:
#                     print("move sub")
#                     transform_search_target("search_target", "camera_link", NODE_ORI_OFFSETS[node_variation_counter if except_flag else node_variation_counter+1], [z_pos, x_pos, y_pos])
#                 sleep(1)
#         finally:
#             print("move main")
            
#             transform_search_target("search_target", "camera_link", NODE_ORI_OFFSETS[0], [z_pos, x_pos, y_pos])

#         rate.sleep()


# #*** Node Starts Here ***#
# if __name__ == "__main__":
#     main()

# """

import rospy

# tf2 and Transformations
import tf2_ros
from math import pi
from time import sleep
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool

# Temp solution - Fix this import to something like `from dual_robot_control.robot_services import RobotControl`
import importlib.util
import sys
spec = importlib.util.spec_from_file_location("RobotControl", "/home/drojas/dlo_ws/src/wire_manipulation/manipulation/dual_robot_control/src/robot_services.py")
RC = importlib.util.module_from_spec(spec)
sys.modules["RobotControl"] = RC
spec.loader.exec_module(RC)

NODE_POS_OFFSETS = { # Specified z, x, y
    0 : [0, 0, 1],
    1 : [0, 1, 1],
    2 : [0, 1, 0],
    3 : [0, 1, -1],
    4 : [0, 0, -1],
    5 : [0, -1, -1],
    6 : [0, -1, 0],
    7 : [0, -1, 1],
}

# Definitions for angles constant across all subnodes
X_ANG = pi/6
Y_ANG = pi/6
NODE_ORI_OFFSETS = { # Specified z, x, y
    0 : [0, 0, 0],
    1 : [0, X_ANG, 0],
    2 : [0, X_ANG, -Y_ANG],
    3 : [0, 0, -Y_ANG],
    4 : [0, -X_ANG, -Y_ANG],
    5 : [0, -X_ANG, 0],
    6 : [0, -X_ANG, Y_ANG],
    7 : [0, 0, Y_ANG],
    8 : [0, X_ANG, Y_ANG],
}

class SearchRoutine():
    def __init__(self, search_arm, grasp_arm) -> None:
        # Robot Control
        self.robot_control = RC.RobotControl()
        # Arm assignments
        self.SEARCHING_ARM = search_arm
        self.GRASPING_ARM  = grasp_arm
        # Transform and frame lookup
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def check_aruco_found(self, target : str, pos, ori):
        # Moves to search target and checks if frame is there
        self.robot_control.move_to_arg(self.SEARCHING_ARM, pos, ori)
        # sleep(2,5) # maybe need a sleep here when testing real?
        try:
            self.tfBuffer.lookup_transform('camera_color_optical_frame', target, rospy.Time(0), rospy.Duration(5))
            return True
        except tf2_ros.LookupException:
            return False

    def transform_search_target(self, child_name: str, source: str, pos_adj, ori_adj) -> None:
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

        return [t.transform.translation.x,t.transform.translation.y,t.transform.translation.z],[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w]
    
    def search(self, ):
        ### Variables for spiral positioning
        # Define max and min for horizontal and vertical coordinates
        # +distance to panels, horizontal 0.325:-0.4, vertical -0.4:0.2
        MIN_X, MAX_X = -0.4, 0.3
        MIN_Y, MAX_Y = -0.4, 0.2
        dx, dy = 0.1, 0.1

        z_pos, x_pos, y_pos, = 0.4, 0.2, 0.25 # initalize search target at middle origin position; updated through search routine

        x_dir, y_dir = -1, 1 # direction to start the search
        x_min_reached, x_max_reached = x_pos,x_pos # both positions must be adjusted to change coordinate (e.g. .2, .2)
        y_min_reached, y_max_reached = y_pos,y_pos

        POS_SAVE = {
            'z' : z_pos,
            'x' : x_pos,
            'y' : y_pos,
        }

        next_dir = 'x'

        SEARCHING = True
        TAG_FOUND = False
        while SEARCHING:
            # z_pos, x_pos, y_pos = POS_SAVE['z'], POS_SAVE['x'], POS_SAVE['y']
            POS_SAVE['z'], POS_SAVE['x'], POS_SAVE['y'] = z_pos, x_pos, y_pos


            pos,ori = self.transform_search_target("search_target", "world", NODE_ORI_OFFSETS[0], [z_pos, x_pos, y_pos])
            if self.check_aruco_found('arm_aruco_0', pos,ori):
                SEARCHING = False # end search when aruco found
                TAG_FOUND = True

            # else: # no aruco found at parallel position (primary node), try all subnodes
            #     node_variation_counter = 0
            #     while node_variation_counter < len(NODE_POS_OFFSETS):
            #         # adjust pos and ori for sub 
            #         x_pos = POS_SAVE['x'] + (NODE_POS_OFFSETS[node_variation_counter][1]*dx/2)
            #         y_pos = POS_SAVE['y'] + (NODE_POS_OFFSETS[node_variation_counter][2]*dy/2)

            #         pos, ori = self.transform_search_target("search_target", "camera_link", NODE_ORI_OFFSETS[node_variation_counter+1], [z_pos, x_pos, y_pos])
            #         if self.check_aruco_found('arm_aruco_0', pos, ori):
            #             SEARCHING = False # end search when aruco found
            #             TAG_FOUND = True
            #             break
            #         else:
            #             node_variation_counter += 1

            z_pos, x_pos, y_pos = POS_SAVE['z'], POS_SAVE['x'], POS_SAVE['y']
            if not TAG_FOUND:                
                if next_dir == 'x':
                    x_delta = x_dir * dx
                    x_adj = x_pos + x_delta
                    if (x_pos <= MAX_X and x_pos >= MIN_X):
                        x_pos = x_adj
                        
                    if (x_dir > 0 and x_pos > x_max_reached):
                        x_max_reached += dx
                        x_dir *= -1
                        next_dir = 'y'
                    elif (x_dir < 0 and x_pos < x_min_reached):
                        x_min_reached -= dx
                        x_dir *= -1
                        next_dir = 'y'

                elif next_dir == 'y':
                    y_delta = y_dir * dy
                    y_adj = y_pos + y_delta
                    if (y_pos <= MAX_Y and y_pos >= MIN_Y):
                        y_pos = y_adj
                        
                    if (y_dir > 0 and y_pos > y_max_reached):
                        y_max_reached += dy
                        y_dir *= -1
                        next_dir = 'x'
                    elif (y_dir < 0 and y_pos < y_min_reached):
                        y_min_reached -= dy
                        y_dir *= -1
                        next_dir = 'x'

        return TAG_FOUND

def main():
    rospy.init_node('search_target')
    
    rate = rospy.Rate(60)

    searchRoutine = SearchRoutine("left", "right")

    searchRoutine.search()

    # while not rospy.is_shutdown():
    #     transform_search_target("search_target", "camera_link", NODE_ORI_OFFSETS[4], [0.7, 0.2, -0.1])

    #     rate.sleep()

#*** Node Starts Here ***#
if __name__ == "__main__":
    main()

# """