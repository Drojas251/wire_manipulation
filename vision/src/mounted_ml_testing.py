#!/usr/bin/env python3
import rospy

# tf2 and Transformations
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo

# Camera capture
from cv_bridge import CvBridge,CvBridgeError
from collections import defaultdict

import cv2
import numpy as np 
import cam_calibration

### RUN THIS FOR FULLY TRAINED YOLOv4 for basic items

# CAMERA_SRC = cv2.VideoCapture(4) # Depth cam device index 4; use when running without ROS
# Load classes for identification in output frame
classes = []
with open("/home/drojas/dlo_ws/src/wire_manipulation/dnn_model/classes.txt", "r") as file_obj:
    for class_name in file_obj.readlines():
        classes.append(class_name.strip())

class MountedMLTracker:
    def __init__(self):
        # Subscribers to Camera
        self.rgb_img_sub = rospy.Subscriber("/mounted_cam/camera/color/image_raw", Image, self.track_callback,queue_size=1)
        
        # Image member variables
        self.bridge_object = CvBridge()
        self.seg_depth_img = Image()
        self.depth_data = []
        self.depth_cam_info = CameraInfo()

        # cv2 ML Models
        self.net = cv2.dnn.readNet("/home/drojas/dlo_ws/src/wire_manipulation/dnn_model/yolov4-tiny.weights", "/home/drojas/dlo_ws/src/wire_manipulation/dnn_model/yolov4-tiny.cfg")
        self.model = cv2.dnn_DetectionModel(self.net)

    def set_model_params(self, size, scale):
        self.model.setInputParams(size=size, scale=scale)

    def track_callback(self, data):
        try:
            frame = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        rospy.sleep(0.01)

        # ret, frame = CAMERA_SRC.read() # If not using ROS
        # Object detection
        class_ids, scores, bboxes = self.model.detect(frame) # category of object, confidence, and location
        for class_id, score, bbox in zip(class_ids, scores,bboxes):
            (x,y,w,h) = bbox
            cv2.putText(frame, str(classes[class_id]), (x,y-10), cv2.FONT_HERSHEY_PLAIN, 2, (200,0,50), 2)
            cv2.rectangle(frame, (x,y), (x+w, y+h), (200,0,50), 3)


        # Display the resulting frame
        resized_frame = cv2.resize(frame, (0,0), fx=0.80, fy=0.80)
        cv2.imshow('Rear Mounted Camera ML', resized_frame) 
        cv2.waitKey(1)

def main():
    rospy.init_node("mounted_aruco_tracker",anonymous=True)
    rospy.sleep(3)

    tracker = MountedMLTracker()
    tracker.set_model_params((320,320), 1/255)


    print(tracker)

    rospy.spin()

if __name__ == '__main__':
    main()
