#!/usr/bin/env python3
import rospy

## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_device('109122071019')

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

# found_rgb = False
# for s in device.sensors:
#     if s.get_info(rs.camera_info.name) == 'RGB Camera':
#         found_rgb = True
#         break
# if not found_rgb:
#     print("The demo requires Depth camera with Color sensor")
#     exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)




# Start streaming
profile = pipeline.start(config)


depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())


        print(depth_image[100,100] * depth_scale)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.3), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)

finally:

    # Stop streaming
    pipeline.stop()

# #!/usr/bin/env python3
# import rospy

# # tf2 and Transformations
# import tf2_ros
# import tf_conversions
# from geometry_msgs.msg import TransformStamped
# from sensor_msgs.msg import Image, CameraInfo

# # Camera capture
# from cv_bridge import CvBridge,CvBridgeError
# from collections import defaultdict

# import cv2
# import numpy as np 
# import cam_calibration

# ### RUN THIS FOR FULLY TRAINED YOLOv4 for basic items

# # CAMERA_SRC = cv2.VideoCapture(4) # Depth cam device index 4; use when running without ROS
# # Load classes for identification in output frame
# classes = []
# with open("/home/drojas/dlo_ws/src/wire_manipulation/dnn_model/classes.txt", "r") as file_obj:
#     for class_name in file_obj.readlines():
#         classes.append(class_name.strip())

# class MountedMLTracker:
#     def __init__(self):
#         # Subscribers to Camera
#         self.rgb_img_sub = rospy.Subscriber("/mounted_cam/camera/color/image_raw", Image, self.track_callback,queue_size=1)
        
#         # Image member variables
#         self.bridge_object = CvBridge()
#         self.seg_depth_img = Image()
#         self.depth_data = []
#         self.depth_cam_info = CameraInfo()

#         # cv2 ML Models
#         self.net = cv2.dnn.readNet("/home/drojas/dlo_ws/src/wire_manipulation/dnn_model/yolov4-tiny.weights", "/home/drojas/dlo_ws/src/wire_manipulation/dnn_model/yolov4-tiny.cfg")
#         self.model = cv2.dnn_DetectionModel(self.net)

#     def set_model_params(self, size, scale):
#         self.model.setInputParams(size=size, scale=scale)

#     def track_callback(self, data):
#         try:
#             frame = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
#         except CvBridgeError as e:
#             print(e)
#         rospy.sleep(0.01)

#         # ret, frame = CAMERA_SRC.read() # If not using ROS
#         # Object detection
#         class_ids, scores, bboxes = self.model.detect(frame) # category of object, confidence, and location
#         for class_id, score, bbox in zip(class_ids, scores,bboxes):
#             (x,y,w,h) = bbox
#             cv2.putText(frame, str(classes[class_id]), (x,y-10), cv2.FONT_HERSHEY_PLAIN, 2, (200,0,50), 2)
#             cv2.rectangle(frame, (x,y), (x+w, y+h), (200,0,50), 3)


#         # Display the resulting frame
#         resized_frame = cv2.resize(frame, (0,0), fx=0.80, fy=0.80)
#         cv2.imshow('Rear Mounted Camera ML', resized_frame) 
#         cv2.waitKey(1)

# def main():
#     rospy.init_node("mounted_aruco_tracker",anonymous=True)
#     rospy.sleep(3)

#     tracker = MountedMLTracker()
#     tracker.set_model_params((320,320), 1/255)


#     print(tracker)

#     rospy.spin()

# if __name__ == '__main__':
#     main()
