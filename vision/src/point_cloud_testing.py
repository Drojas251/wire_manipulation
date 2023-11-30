#!/usr/bin/env python3
import rospy
#import pcl
import numpy as np
import ros_numpy
from std_msgs.msg import Header

from sensor_msgs.msg import PointCloud2, PointField
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

from ctypes import * # convert float to uint32

### Conversion helpers from https://github.com/felixchenfy/open3d_ros_pointcloud_conversion/blob/master/lib_cloud_conversion_between_Open3D_and_ROS.py
# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)

# Convert the datatype of point cloud from Open3D to ROS PointCloud2 (XYZRGB only)
def convertCloudFromOpen3dToRos(open3d_cloud, frame_id="odom"):
    # Set "header"
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Set "fields" and "cloud_data"
    points=np.asarray(open3d_cloud.points)
    if not open3d_cloud.colors: # XYZ only
        fields=FIELDS_XYZ
        cloud_data=points
    else: # XYZ + RGB
        fields=FIELDS_XYZRGB
        # -- Change rgb color from "three float" to "one 24-byte int"
        # 0x00FFFFFF is white, 0x00000000 is black.
        colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
        colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2]  
        cloud_data=np.c_[points, colors]
    
    # create ros_cloud
    return pc2.create_cloud(header, fields, cloud_data)

def convertCloudFromRosToOpen3d(ros_cloud):
    
    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    # Check empty
    open3d_cloud = o3d.PointCloud()
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3 # x, y, z, rgb
        
        # Get xyz
        xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

        # combine
        open3d_cloud.points = o3d.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = o3d.Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
        open3d_cloud.points = o3d.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud

class MatchCAD():
    def __init__(self) -> None:
        # Subscribers to Nodelet manager
        self.pc = rospy.Subscriber("/rscamera/depth/points", PointCloud2, self.pc_callback)
        # /mounted_cam/camera/depth/color/points

        self.vis = o3d.visualization.Visualizer()
        self.pc = None

    def pc_callback(self, data):
        # points are in camera_color_optical_frame
        self.pc = convertCloudFromRosToOpen3d(data)
        # if self.pc is not None:
        self.points = o3d.utility.Vector3dVector(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self))
        self.vis.create_window()

        self.vis.add_geometry(self)
        
        self.vis.update_geometry()
        self.vis.poll_events()

        self.vis.update_renderer()

    def update_vis(self):
        self.pc.points =  o3d.utility.Vector3dVector(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.pc))
        self.vis.update_geometry()
        self.vis.poll_events()
        self.vis.update_renderer()

def main():
    rospy.init_node("point_cloud_testing",anonymous=True)
    rospy.sleep(3)

    rate = rospy.Rate(60)
    
    tracker = MatchCAD()
    # while not rospy.is_shutdown():
    #     tracker.update_vis()

    #     rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    main()
