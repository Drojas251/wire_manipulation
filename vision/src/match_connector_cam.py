#!/usr/bin/env python3
from time import sleep
import rospy

from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion

import numpy as np
import open3d as o3d

import ros_numpy
from stl import mesh

from mpl_toolkits import mplot3d
from matplotlib import pyplot

# Transform publishing
from tf.transformations import quaternion_from_euler, quaternion_about_axis
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math

rospy.init_node("match_connector_cam",anonymous=True)
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

def transform_connector_match_cam(child_name: str, source: str, cam_spec, pos_adj, ori_adj):
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = source
        t.child_frame_id = "connector_{}".format(cam_spec)

        t.transform.translation.x = ori_adj[0]
        t.transform.translation.y = ori_adj[1] 
        t.transform.translation.z = ori_adj[2] #+ 0.25

        transform = None
        try:
            transform = tf_buffer.lookup_transform(source, cam_spec, rospy.Time())
            rotation = transform.transform.rotation

            # transform2 = tf_buffer.lookup_transform("a_bot_ee_arm_link", cam_spec, rospy.Time())
            # print(transform2.transform.rotation)
        except Exception as e:
            print(e)
        if not transform:
             return

        # q = quaternion_from_euler(pos_adj[0], pos_adj[1], pos_adj[2]) # pos_adj
        # # q = quaternion_from_euler(-math.pi/2,math.pi/2,0) # match rotation of bot grippers

        """
        Combinations: 
        - y/w need to flip 180 (worst)
        - z/y move blue up pi/2
        - xz exactly upright
        - zw flip 180
        """
        pitch = math.atan2(rotation.x, rotation.z)  

        q = quaternion_from_euler(0, pitch, 0) # match rotation of bot grippers
        # print(q,'\n')
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # t.transform.rotation.x = rotation.x
        # t.transform.rotation.y = rotation.y
        # t.transform.rotation.z = rotation.z
        # t.transform.rotation.w = rotation.w

        br.sendTransform(t)

def main():
    rate = rospy.Rate(60)
    rear_cam_spec = "mounted_cam"
    arm_cam_spec = "arm_cam"

    while not rospy.is_shutdown():
        transform_connector_match_cam("adj_grasp_mounted_cam", "line_grasp_mounted_cam", "d415_color_frame", [0,0,0], [0, 0, 0, 1])

if __name__ == '__main__':
    main()
