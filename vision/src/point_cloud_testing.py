#!/usr/bin/env python3
import rospy
#import pcl
import numpy as np
import ros_numpy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import math

from sensor_msgs.msg import Image, CameraInfo
import pyrealsense2 as rs2
# Camera capture
from cv_bridge import CvBridge,CvBridgeError

import cv2
import numpy as np 

import pyrealsense2 as ps2
import open3d as o3d

class MatchCAD():
    def __init__(self) -> None:
        # Subscribers to Camera
        self.rgb_img_sub = rospy.Subscriber("/mounted_cam/camera/color/image_raw", Image, self.track_callback,queue_size=1)
        self.depth_img_sub = rospy.Subscriber("/mounted_cam/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        self.depth_cam_info = rospy.Subscriber("/mounted_cam/camera/aligned_depth_to_color/camera_info",CameraInfo, self.depth_cam_info_callback)

        # Image member variables
        self.bridge = CvBridge()
        self.depth_image = []
        self.depth_cam_info = CameraInfo()
        self.intrinsics = None

        # rospy.Subscriber("/rscamera/depth/points", PointCloud2, self.pc_callback)

    def pc_callback(self, data):
        pc = ros_numpy.numpify(data)
        points=np.zeros((len(pc)*len(pc[0]),3))
        count = 0
        N = 20

        print(pc)

        # translation = np.array([0, 0, 0])

        # # format the processed pointcloud into a N x 3 numpy array
        # for x in range(len(pc)):
        #     for y in range(len(pc[0])):
        #         points2 = pc[x,y]
        #         PP = [points2[0],points2[1],points2[2]]
        #         if not math.isnan(points2[1]):
        #             # transform the points into the world frame
        #             points[count] = [points2[0],points2[1],points2[2]] + translation  
        #             count = count + 1 

def main():
    rospy.init_node("point_cloud_testing",anonymous=True)
    rospy.sleep(3)

    rate = rospy.Rate(60)
    tracker = MatchCAD()
    # while not rospy.is_shutdown():
    #     ml_src = "camera_link" # camera_aligned_depth_to_color_frame
    #     tracker.transform_ml_end(ml_src, [0, 0, 0], [0, 0, 0, 1])

    #     rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    main()
