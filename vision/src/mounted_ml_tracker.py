#!/usr/bin/env python3
import rospy

# tf2 and Transformations
import tf2_ros
import tf_conversions
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo
import pyrealsense2

# Camera capture
from cv_bridge import CvBridge,CvBridgeError

import cv2
import numpy as np 

from roboflow import Roboflow
rf = Roboflow(api_key="JZTKTAQvOFKLZTZdUNhR")
project = rf.workspace().project("deformable-linear-objects-connector-detection")
model = project.version(1).model

class MountedMLTracker:
    def __init__(self):
        # Subscribers to Camera
        self.rgb_img_sub = rospy.Subscriber("/mounted_cam/camera/color/image_raw", Image, self.track_callback,queue_size=1)
        
        # Image member variables
        self.bridge_object = CvBridge()
        self.seg_depth_img = Image()
        self.depth_data = []
        self.depth_cam_info = CameraInfo()

        self.x = 0
        self.y = 0
        self.end_class = None
        self.converted_pt = [0.0, 0.0, 0.0]

    def convert_depth_to_phys_coord_using_realsense(self, depth : float, cam : str = "rear"):
        if cam == "rear":
            cameraInfo = rospy.wait_for_message("/mounted_cam/camera/color/camera_info", CameraInfo)
        else:
            cameraInfo = rospy.wait_for_message("/arm_cam/camera/color/camera_info", CameraInfo)

        _intrinsics = pyrealsense2.intrinsics()
        _intrinsics.width = cameraInfo.width
        _intrinsics.height = cameraInfo.height
        _intrinsics.ppx = cameraInfo.K[2]
        _intrinsics.ppy = cameraInfo.K[5]
        _intrinsics.fx = cameraInfo.K[0]
        _intrinsics.fy = cameraInfo.K[4]
        # _intrinsics.model = cameraInfo.distortion_model
        _intrinsics.model  = pyrealsense2.distortion.none
        _intrinsics.coeffs = [i for i in cameraInfo.D]

        result = pyrealsense2.rs2_deproject_pixel_to_point(_intrinsics, [self.x, self.y], depth)  #result[0]: right, result[1]: down, result[2]: forward
        return [result[2], -result[0], -result[1]]
    
    def track_callback(self, data):
        try:
            frame = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        rospy.sleep(0.01)

        # infer on a local image
        predictions = model.predict(frame, confidence=40, overlap=30).json()
        self.x = predictions["predictions"][0]['x']
        self.y = predictions["predictions"][0]['y']
        self.end_class = predictions["predictions"][0]["class"]
        self.converted_pt = self.convert_depth_to_phys_coord_using_realsense(0.6858)

        # Add bounding boxes to the model prediction of connector
        for bounding_box in predictions["predictions"]:
            x0 = bounding_box['x'] - bounding_box['width']  / 2
            x1 = bounding_box['x'] + bounding_box['width']  / 2
            y0 = bounding_box['y'] - bounding_box['height'] / 2
            y1 = bounding_box['y'] + bounding_box['height'] / 2
            
            start_point = (int(x0), int(y0))
            endpoint = (int(x1), int(y1))
            cv2.rectangle(frame, start_point, endpoint, color=(0,255,0), thickness=2)
        
        # Display the resulting frame
        resized_frame = cv2.resize(frame, (0,0), fx=0.80, fy=0.80)
        cv2.imshow('Rear Mounted Camera ML', resized_frame) 
        cv2.waitKey(1)

    def transform_ml_end(self, source: str, pos_adj, ori_adj) -> None:
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = source
        t.child_frame_id = str(self.end_class)

        print(self.converted_pt)
        t.transform.translation.x = self.converted_pt[0] #ori_adj[0] # Offset arm to right by value meters
        t.transform.translation.y = self.converted_pt[1] #ori_adj[1]
        t.transform.translation.z = self.converted_pt[2] #ori_adj[2] # Too close to wall, move back .05m

        ### For now ignore rotation
        q = quaternion_from_euler(pos_adj[0], pos_adj[1], pos_adj[2]) # pos_adj
        # q = quaternion_from_euler(-math.pi/2,math.pi/2,0) # match rotation of bot grippers

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)

def main():
    rospy.init_node("mounted_aruco_tracker",anonymous=True)
    rospy.sleep(3)

    rate = rospy.Rate(60)
    tracker = MountedMLTracker()
    while not rospy.is_shutdown():
        tracker.transform_ml_end("world", [0, 0, 0], [0, 0, 0, 1])

        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
