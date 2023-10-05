#!/usr/bin/env python3
import rospy

from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import pcl
import numpy as np
import open3d as o3d

import ros_numpy
from stl import mesh

from mpl_toolkits import mplot3d
from matplotlib import pyplot

# Transform publishing
from tf.transformations import quaternion_from_euler
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math

class ConnectorPC():
    def __init__(self) -> None:
        # Subscriber to pointcloud from camera
        self.points_sub = rospy.Subscriber("/rscamera/depth_image/points", PointCloud2, self.pc_callback, queue_size=1)

        # Publisher of ICP pointclouds
        self.icp_pub = rospy.Publisher('/aligned_icp_pc', PointCloud2, queue_size=10)

        # Publisher of fitted shape to pointcloud
        self.shape_pub = rospy.Publisher('/pc_shape', Marker, queue_size=100)
        self.line_pub = rospy.Publisher('/pc_line', Marker, queue_size=10)

        # # Fitted shape params
        # self.shape_params = None

        # # Store a collected PC
        self.source_pc = None
        self.numpy_pc = None

        # Store best fit lines
        self.lsr_line = None
        self.ransac_line = None

    ### Callbacks
    def pc_callback(self, points):
        # Source pointcloud and translated copy
        self.source_pc = points
        self.numpy_pc  = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(points)

    def translate_pc(self, input_pc, translation):
        # Only used to translate a pc x,y,z units; originally for ICP testing between source and its translation
        translated_pc = []

        for p in pc2.read_points(input_pc, field_names=("x","y","z"), skip_nans=True):
            p = [p[0] + translation[0], p[1] + translation[1], p[2] + translation[2]]
            
            translated_pc.append(p)
            # pc2.write_points(translated_pc, [p])

        return pc2.create_cloud_xyz32(input_pc.header, translated_pc)
    
    def calc_icp(self, source_pcd, target_pcd):
        # Performs ICP registration
        icp = o3d.pipelines.registration.registration_icp(
            source_pcd, target_pcd, max_correspondence_distance=0.1,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )

        # Apply the transformation to the source PointCloud
        source_pcd.transform(icp.transformation)

        return source_pcd

    def ransac_fit(self):
        pass
    
    ### Visualization helper functions
    def visualize_shape(self, src_frame, center, radius, axis):
        marker = Marker()
        marker.header.frame_id = src_frame
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = center[0]  # Center of the cylinder
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]

        marker.pose.orientation.x = 0.0 # axis?
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 2 * radius  # Diameter of the cylinder
        marker.scale.y = 2 * radius  # Diameter of the cylinder
        marker.scale.z = radius * 10  # Height of the cylinder - not yet known
        marker.color.a = 0.5  # Transparency
        marker.color.r = 1.0  # Red color
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Publish the marker
        self.shape_pub.publish(marker)

    def visualize_cad_stl(self, path):
        ### Visualizer using pyplot for a CAD model specified by path
        # Create a new plot
        figure = pyplot.figure()
        axes = figure.add_subplot(projection='3d')

        # Load the STL files and add the vectors to the plot
        your_mesh = mesh.Mesh.from_file(path)
        axes.add_collection3d(mplot3d.art3d.Poly3DCollection(your_mesh.vectors))

        # Auto scale to the mesh size
        scale = your_mesh.points.flatten()
        axes.auto_scale_xyz(scale, scale, scale)

        # Show the plot to the screen
        pyplot.show()

    def visualize_line(self, src_frame):
        # Define and color the line
        line_marker = Marker()
        line_marker.header.frame_id = src_frame
        line_marker.type = Marker.LINE_STRIP  # Type for line strip
        line_marker.action = Marker.ADD  # Add a new marker
        line_marker.scale.x = 0.01  # Line width
        line_marker.color.r = 1.0  # Red color
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0  # Alpha (transparency)

        # Replace these with your line coefficients and desired endpoints
        """
        Define the endpoints of your line by adding points to the points field of the Marker. 
        In your case, you have the coefficients of a geometric line, so you need to calculate 
        the two points that define the line:
        """
        # use self.source_pc thats equivalent to points
        min_x_pt, max_x_pt = None, None
        # calculate min and max points
        for pt in pc2.read_points(self.source_pc, field_names=("x", "y", "z"), skip_nans=True):
            pt_x = depth = pt[0]
            if min_x_pt == None:
                min_x_pt = pt
            else:
                min_x_pt = pt if pt_x < min_x_pt[0] else min_x_pt

            if max_x_pt == None:
                max_x_pt = pt
            else:
                max_x_pt = pt if pt_x > max_x_pt[0] else max_x_pt

        x1, y1, z1 = min_x_pt[0], min_x_pt[1], min_x_pt[2]  # Start point
        x2, y2, z2 = max_x_pt[0], max_x_pt[1], max_x_pt[2]  # End point

        # Create Point messages for the start and end points
        start_point = Point(x1, y1, z1)
        end_point = Point(x2, y2, z2)

        # Add the points to the Marker's points field
        line_marker.points.append(start_point)
        line_marker.points.append(end_point)

        # Publish the marker
        self.line_pub.publish(line_marker)

    ### Getters/setters
    def get_src_pc(self):
        return self.source_pc
    
    def get_numpy_pc(self):
        return self.numpy_pc

    ### Conversion helpers
    def np_array_to_open3d(self, array):
        # Create an Open3D PointCloud object
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(array)

        return pcd
    
    def np_array_to_ros_pointcloud(self, frame_id, points):
        msg = PointCloud2()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id

        msg.height = 1
        msg.width = points.shape[0]
        msg.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = np.asarray(points, np.float32).tobytes()
        
        return msg

    def ros_pointcloud_to_open3d(self, pointcloud_msg):
        # Extract point data from the ROS PointCloud2 message
        points = pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True)
        pointcloud = np.array(list(points), dtype=np.float32)

        # Create an Open3D PointCloud object
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pointcloud)

        return pcd

    def ros_pointcloud_to_np_array(self, pointcloud_msg):
        points = pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True)
        return np.array(list(points), dtype=np.float32)

    def open3d_pointcloud_to_ros(self, pcd, original_msg):
        # Get the header information from the original PointCloud2 message
        header = original_msg.header

        # Convert Open3D PointCloud to a numpy array
        points = np.asarray(pcd.points)

        # Create a new ROS PointCloud2 message
        new_msg = pc2.create_cloud_xyz32(header, points)

        return new_msg

    ### Fitting
    def fit_line_least_squares_regression(self):
        pointcloud = self.numpy_pc

        # Calc mean for each dimension of pc
        x_mean = np.mean(pointcloud[:, 0])
        y_mean = np.mean(pointcloud[:, 1])
        z_mean = np.mean(pointcloud[:, 2])

        # Calc covariance matrix of pc
        covariance_matrix = np.cov(pointcloud.T)

        # Calc eigenvectors and eigenvalues of the covariance matrix
        eigenvalues, eigenvectors = np.linalg.eig(covariance_matrix)

        # Eigenvector with the smallest eigenvalue corresponds to the direction of the line
        smallest_eigenvalue_idx = np.argmin(eigenvalues)
        line_direction = eigenvectors[:, smallest_eigenvalue_idx]

        # Calculate the coefficients of the line
        a, b, c = line_direction
        d = -(a*x_mean + b*y_mean + c*z_mean)

        return a, b, c, d

    def fit_line_ransac(self, n_iterations=100, threshold=0.1):
        pointcloud = self.numpy_pc

        best_line = None
        best_inliers_count = 0
        
        for i in range(n_iterations):
            # Step 1: Randomly select two points from the pointcloud
            indices = np.random.choice(len(pointcloud), 2)
            p1, p2 = pointcloud[indices]

            # Step 2: Fit a line through the two points
            line_direction = p2 - p1

            # Calculate the coefficients of the line
            a, b, c = line_direction
            d = -(a*p1[0] + b*p1[1] + c*p1[2])

            # Step 3: Compute the residual errors for all the points in the pointcloud
            distances = np.abs(pointcloud.dot(np.array([a,b,c])) + d) / np.sqrt(a**2 + b**2 + c**2)

            # Step 4: Count the number of inliers and update the best-fitting line if necessary
            inliers_count = np.sum(distances < threshold)
            if inliers_count > best_inliers_count:
                best_inliers_count = inliers_count
                best_line = a, b, c, d

        return best_line

    def fit_cylinder(self):
        
        if self.numpy_pc is None or not self.numpy_pc.any():
            return
        # cylinder = pyrsc.Cylinder()
        # center, axis, radius, inliers = cylinder.fit(self.numpy_pc, thresh=0.4)

        # """
        # This is a fitting for a vertical cylinder fitting
        # Reference:
        # http://www.int-arch-photogramm-remote-sens-spatial-inf-sci.net/XXXIX-B5/169/2012/isprsarchives-XXXIX-B5-169-2012.pdf

        # xyz is a matrix contain at least 5 rows, and each row stores x y z of a cylindrical surface
        # p is initial values of the parameter;
        # p[0] = Xc, x coordinate of the cylinder centre
        # P[1] = Yc, y coordinate of the cylinder centre
        # P[2] = alpha, rotation angle (radian) about the x-axis
        # P[3] = beta, rotation angle (radian) about the y-axis
        # P[4] = r, radius of the cylinder

        # th, threshold for the convergence of the least squares

        # """   
        # x = self.numpy_pc[:,0]
        # y = self.numpy_pc[:,1]
        # z = self.numpy_pc[:,2]
        # p = np.array([0,0,0,0,0])

        # fitfunc = lambda p, x, y, z: (- np.cos(p[3])*(p[0] - x) - z*np.cos(p[2])*np.sin(p[3]) - np.sin(p[2])*np.sin(p[3])*(p[1] - y))**2 + (z*np.sin(p[2]) - np.cos(p[2])*(p[1] - y))**2 #fit function
        # errfunc = lambda p, x, y, z: fitfunc(p, x, y, z) - p[4]**2 #error function 

        # est_p , success = leastsq(errfunc, p, args=(x, y, z), maxfev=1000)

        # return est_p

        # print(f"center: {center}\naxis: {axis}\nradius: {radius}\n")
        # self.visualize_shape("camera_color_optical_frame", center, radius*.05, axis)

    ### Orientation calculation
    def normalize_vector(self, vector):
        if vector:
            magnitude = math.sqrt(vector[0]**2 + vector[1]**2 + vector[2]**2)
            normalized_vector = [vector[0] / magnitude, vector[1] / magnitude, vector[2] / magnitude]
            return normalized_vector
        else:
            return [0,0,0,1]


    ### Publish full pose
    def transform_connector_pose(self, child_name: str, source: str, pos_adj, ori_adj) -> None:
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = source
        t.child_frame_id = "{}_{}".format(child_name, source)

        t.transform.translation.x = ori_adj[0] # Offset arm to right by value meters
        t.transform.translation.y = ori_adj[1]
        t.transform.translation.z = ori_adj[2] # Too close to wall, move back .05m

        q = quaternion_from_euler(pos_adj[0], pos_adj[1], pos_adj[2]) # pos_adj
        # q = quaternion_from_euler(-math.pi/2,math.pi/2,0) # match rotation of bot grippers

        norm_vec = self.normalize_vector(self.ransac_line)
        t.transform.rotation.x = q[0] + norm_vec[0]
        t.transform.rotation.y = q[1] + norm_vec[1]
        t.transform.rotation.z = q[2] + norm_vec[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)

    ### Testing functions
    def test_icp_pc_translation(self):
        ### This was just used for testing ICP between src and translated pc
        points = self.source_pc
        if points == None:
            return None
        
        # transf_pc = self.translate_pc(points, [1, 1, 1])
        # Attempt ICP with CAD
        stl_path = "vision/resources/cad/cylinder.stl"
        stl_mesh = mesh.Mesh.from_file(stl_path)
        vertices = stl_mesh.vectors.reshape((-1,3))
        unique_vertices = np.unique(vertices, axis=0)

        # Convert to Open3d format
        o3d_source = self.ros_pointcloud_to_open3d(points)
        o3d_transf = self.np_array_to_open3d(unique_vertices)
        # o3d_transf = self.ros_pointcloud_to_open3d(transf_pc)

        # Perform ICP
        aligned_pc = self.calc_icp(o3d_source, o3d_transf)

        # Convert back to ROS pointcloud type
        pc2_aligned = self.open3d_pointcloud_to_ros(aligned_pc, points)

        # self.visualize_cad_stl(stl_path)

        # Publish for viewing in Rviz
        self.icp_pub.publish(pc2_aligned)
        # self.icp_pub.publish(self.np_array_to_ros_pointcloud("camera_color_optical_frame", unique_vertices))

        return pc2_aligned

    def test_line_fitting(self):
        if self.numpy_pc is None or not self.numpy_pc.any():
            return
        
        self.lsr_line = self.fit_line_least_squares_regression()
        self.ransac_line = self.fit_line_ransac()

        # print(f"Least Squares Regression:\n{lsr_line}")
        # print(f"RANSAC:\n{ransac_line}")
        # print()
        self.visualize_line("camera_color_optical_frame")

def main():
    rospy.init_node("proc_connector_pc",anonymous=True)
    rospy.sleep(3)

    # rate = rospy.Rate(60)
    proc_pc = ConnectorPC()
    while not rospy.is_shutdown():
        # proc_pc.test_icp_pc_translation()
        proc_pc.test_line_fitting()
        proc_pc.transform_connector_pose("cpose", "usb-crotation", [0,0,0], [0,0,0,1])

        # if proc_pc.get_src_pc():
        #     print("ATTEMPT FIT")
        #     proc_pc.fit_shape()
    # rate.sleep()
    

if __name__ == '__main__':
    main()
