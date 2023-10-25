#!/usr/bin/env python3
import rospy

# tf2 and Transformations
import tf2_ros
import tf_conversions
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import pyrealsense2 as rs2


# Camera capture
from cv_bridge import CvBridge,CvBridgeError

import cv2
import numpy as np 

from roboflow import Roboflow
rf = Roboflow(api_key="JZTKTAQvOFKLZTZdUNhR")
project = rf.workspace().project("deformable-linear-objects-connector-detection")
model = project.version(1).model # 5 broken?

class MountedMLTracker:
    def __init__(self, cam_spec : str = "mounted_cam"):
        # Subscribers to Camera
        self.rgb_img_sub = rospy.Subscriber(f"/{cam_spec}/camera/color/image_raw", Image, self.track_callback,queue_size=1)
        self.depth_img_sub = rospy.Subscriber(f"/{cam_spec}/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback) # use for rgb pixel lookup
        self.depth_cam_info = rospy.Subscriber(f"/{cam_spec}/camera/aligned_depth_to_color/camera_info",CameraInfo, self.depth_cam_info_callback)
        self.segmented_depth_sub = rospy.Subscriber(f"/{cam_spec}/rscamera/depth_image/points", PointCloud2, self.segmented_depth_callback, queue_size=1)

        # Image member variables
        self.bridge = CvBridge()
        self.depth_image = []
        self.depth_cam_info = CameraInfo()
        self.intrinsics = None
        self.cam_spec = cam_spec
        self.cam_name = "Arm" if cam_spec == "arm_cam" else "Rear"

        self.x = 0
        self.y = 0
        self.end_class = None
        self.converted_depth = 0.5 # do a waitformessage?
        self.depth_adjustment = 0.05

        self.converted_pt = [0.0, 0.0, 0.0]

    def convert_image_3d_point(self, depth : float):
        if self.intrinsics:
            result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [self.x, self.y], depth)  #result[0]: right, result[1]: down, result[2]: forward
            return [result[2], -result[0], -result[1]]
    
    def segmented_depth_callback(self, msg):
        sum_pt = 0.0
        num_pt = 0

        # Iterate through the points in the PointCloud2 message
        for pt in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            depth = pt[2]  # Depth is the Z coordinate
            sum_pt += depth
            num_pt += 1

        if num_pt > 0:
            self.converted_depth = ( sum_pt / num_pt ) + self.depth_adjustment
            

    def track_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        rospy.sleep(0.01)
        
        # infer on a local image
        predictions = model.predict(frame, confidence=40, overlap=30).json()
        if not predictions['predictions']:
            return
        
        self.x = predictions["predictions"][0]['x']
        self.y = predictions["predictions"][0]['y']
        self.end_class = predictions["predictions"][0]["class"]
        self.converted_pt = self.convert_image_3d_point(self.converted_depth) # THIS SHOULD BE OUR CALCULATED DEPTH

        # Add bounding boxes to the model prediction of connector
        for bounding_box in predictions["predictions"]:
            x0 = bounding_box['x'] - bounding_box['width']  / 2
            x1 = bounding_box['x'] + bounding_box['width']  / 2
            y0 = bounding_box['y'] - bounding_box['height'] / 2
            y1 = bounding_box['y'] + bounding_box['height'] / 2
            
            start_point = (int(x0), int(y0))
            endpoint = (int(x1), int(y1))
            cv2.rectangle(frame, start_point, endpoint, color=(0,255,0), thickness=2)
        
        # # Display the resulting frame
        # resized_frame = cv2.resize(frame, (0,0), fx=0.80, fy=0.80)
        # cv2.imshow(f'{self.cam_name} Mounted Camera ML', resized_frame) 
        # cv2.waitKey(1)

    def depth_cam_info_callback( self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]

        except CvBridgeError as e:
            print(e)
            return
    
    def depth_callback(self,data):
        # use in rgb lookup approach
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            depth_image_meters = cv_image.astype(np.float32) / 1000.0 # Convert to meters
            
            # depth_at_xy = depth_image_meters[int(self.x), int(self.y)]
            # if depth_at_xy > 0:
                # Only assign if depth > 0, otherwise depth breaks point so leave at 0.5 starting val
                # self.converted_depth = depth_at_xy
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return
    
    def transform_ml_end(self, pos_adj, ori_adj) -> None:
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "d435i_color_frame" if self.cam_spec == "arm_cam" else "d415_color_frame"
        t.child_frame_id = f"{str(self.end_class)}_{self.cam_spec}"

        t.transform.translation.x = self.converted_pt[0] + pos_adj[0]
        t.transform.translation.y = self.converted_pt[1] + pos_adj[1]
        t.transform.translation.z = self.converted_pt[2] + pos_adj[2]
        
        ### For now ignore rotation
        q = quaternion_from_euler(pos_adj[0], pos_adj[1], pos_adj[2]) # pos_adj
        # q = quaternion_from_euler(-math.pi/2,math.pi/2,0) # match rotation of bot grippers

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)

def main():
    rospy.init_node("ml_tracker",anonymous=True)
    rospy.sleep(3)
    rate = rospy.Rate(60)

    # rear_tracker = MountedMLTracker("mounted_cam")
    arm_tracker = MountedMLTracker("arm_cam")
    while not rospy.is_shutdown():
        # z, x, y
        # rear_tracker.transform_ml_end([-0.05, 0, 0.025], [0, 0, 0, 1])
        arm_tracker.transform_ml_end([-0.05, 0, 0], [0, 0, 0, 1])

        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
