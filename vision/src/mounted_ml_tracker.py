#!/usr/bin/env python3
import rospy

# tf2 and Transformations
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo

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

    def track_callback(self, data):
        try:
            frame = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        rospy.sleep(0.01)

        # infer on a local image
        predictions = model.predict(frame, confidence=40, overlap=30).json()
        # Print prediction
        print(predictions)

        # Add bounding boxes to the model prediction of connector
        for bounding_box in predictions["predictions"]:
            x0 = bounding_box['x'] - bounding_box['width'] / 2
            x1 = bounding_box['x'] + bounding_box['width'] / 2
            y0 = bounding_box['y'] - bounding_box['height'] / 2
            y1 = bounding_box['y'] + bounding_box['height'] / 2
            
            start_point = (int(x0), int(y0))
            end_point = (int(x1), int(y1))
            cv2.rectangle(frame, start_point, end_point, color=(0,255,0), thickness=2)
        
        # Display the resulting frame
        resized_frame = cv2.resize(frame, (0,0), fx=0.80, fy=0.80)
        cv2.imshow('Rear Mounted Camera ML', resized_frame) 
        cv2.waitKey(1)

def main():
    rospy.init_node("mounted_aruco_tracker",anonymous=True)
    rospy.sleep(3)

    tracker = MountedMLTracker()

    rospy.spin()

if __name__ == '__main__':
    main()
