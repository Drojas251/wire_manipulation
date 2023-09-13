#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import pcl
import numpy as np
import open3d as o3d
from ctypes import Structure

class ConnectorPC():
    def __init__(self) -> None:
        # Subscriber to pointcloud from camera
        self.points_sub = rospy.Subscriber("/rscamera/depth/points", PointCloud2, self.pc_callback, queue_size=1)

        # Publisher of ICP pointclouds
        self.icp_pub = rospy.Publisher('/aligned_icp_pc', PointCloud2, queue_size=10)

        # Store a collected PC
        self.source_pc = None


    def pc_callback(self, points):
        # Source pointcloud and transformed copy
        self.source_pc = points

    def calc_icp_pc(self):
        points = self.source_pc
        if points == None:
            return None
        
        transf_pc = self.transform_pc(points, [1, 1, 1])

        # Convert to Open3d format
        o3d_source = self.ros_pointcloud_to_open3d(points)
        o3d_transf = self.ros_pointcloud_to_open3d(transf_pc)

        # Perform ICP
        aligned_pc = self.align_point_clouds(o3d_source, o3d_transf)

        # Convert back to ROS pointcloud type
        pc2_aligned = self.open3d_pointcloud_to_ros(aligned_pc, points)

        # Publish for viewing in Rviz
        self.icp_pub.publish(pc2_aligned)

        return pc2_aligned

    def transform_pc(self, input_pc, translation):
        transformed_pc = []

        for p in pc2.read_points(input_pc, field_names=("x","y","z"), skip_nans=True):
            p = [p[0] + translation[0], p[1] + translation[1], p[2] + translation[2]]
            
            transformed_pc.append(p)
            # pc2.write_points(transformed_pc, [p])

        return pc2.create_cloud_xyz32(input_pc.header, transformed_pc)
    
    def align_point_clouds(self, source_pcd, target_pcd):
        # Perform ICP registration
        icp = o3d.pipelines.registration.registration_icp(
            source_pcd, target_pcd, max_correspondence_distance=0.1,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )

        # Apply the transformation to the source PointCloud
        source_pcd.transform(icp.transformation)

        return source_pcd

    
    def draw_registration_result(self, source, target, transformation):
        source_temp = source.clone()
        target_temp = target.clone()

        source_temp.transofrm(transformation)
        # o3d.visualization
    
    ### Conversion helpers
    def ros_pointcloud_to_open3d(self, pointcloud_msg):
        # Extract point data from the ROS PointCloud2 message
        points = pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True)
        pointcloud = np.array(list(points), dtype=np.float32)

        # Create an Open3D PointCloud object
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pointcloud)

        return pcd

    def open3d_pointcloud_to_ros(self, pcd, original_msg):
        # Get the header information from the original PointCloud2 message
        header = original_msg.header

        # Convert Open3D PointCloud to a numpy array
        points = np.asarray(pcd.points)

        # Create a new ROS PointCloud2 message
        new_msg = pc2.create_cloud_xyz32(header, points)

        return new_msg
        
def main():
    rospy.init_node("proc_connector_pc",anonymous=True)
    rospy.sleep(3)

    rate = rospy.Rate(60)
    proc_pc = ConnectorPC()
    while not rospy.is_shutdown():
        proc_pc.calc_icp_pc()
        # USE THIS FRAME AS GRID CORNER, ITERATE FROM THERE??
        rate.sleep()
    

if __name__ == '__main__':
    main()
