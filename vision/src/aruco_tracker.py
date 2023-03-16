#!/usr/bin/env python3
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose # USE THIS
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge,CvBridgeError
from collections import defaultdict

import cv2
import numpy as np 
import cam_calibration

# CAMERA_SRC = cv2.VideoCapture(4) # Depth cam device index 4; use when running without ROS

class ArucoTracker:
    def __init__(self, matrix_coefficients, distortion_coefficients):
        # Subscribers to Camera
        self.aligned_depth_rgb_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.get_depth_data,queue_size=1)
        self.rgb_img_sub = rospy.Subscriber("/camera/color/image_raw",Image, self.track_callback,queue_size=1)
        self.depth_img_camera_info = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info",CameraInfo, self.depth_cam_info_callback,queue_size=1)
        
        # Image member variables
        self.bridge_object = CvBridge()
        self.seg_depth_img = Image()
        self.depth_data = []
        self.depth_cam_info = CameraInfo()

        # Image Coefficients
        self.matrix_coefficients = matrix_coefficients
        self.distortion_coefficients = distortion_coefficients

        # Track result vectors for each id tag found
        self.marker_dict = defaultdict(dict)
        # tvec is 3d position difference between the camera and the marker
        # rvec is Rodriguez's angles between the camera and marker center

        # Publishers for returned ArUco information
        self.aruco_pub = rospy.Publisher("/aruco_position", Float32MultiArray, queue_size=1)

    def next_img(self):
        self.rgb_img_sub = rospy.Subscriber("/camera/color/image_raw",Image, self.track_callback,queue_size=1)
        print(self.marker_dict)

    def track_callback(self, data):
        try:
            frame = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        rospy.sleep(0.01)

        # ret, frame = CAMERA_SRC.read() # If not using ROS
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        parameters =  cv2.aruco.DetectorParameters()
        # lists of ids and the corners beloning to each id
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, aruco_dict,
                                                                parameters=parameters)
        
        if np.all(ids is not None):  # If there are markers found by detector
            for i in range(0, len(ids)):  # Iterate in markers
                # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.127, self.matrix_coefficients,
                                                                        self.distortion_coefficients)
                (rvec - tvec).any()  # Remove numpy value array error
                self.marker_dict[i]["tvec"] = tvec
                self.marker_dict[i]["rvec"] = rvec
                print("tvec:",tvec)
                
                # Publish this?
                # print(list(tvec[0][0]))
                
                cv2.aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
                cv2.drawFrameAxes(frame, self.matrix_coefficients, self.distortion_coefficients, rvec, tvec, .2) 

        # # Display the resulting frame
        cv2.imshow('frame', frame) # 
        cv2.waitKey(1)

    def get_depth_data(self,data):
        cv_depth_image = self.bridge_object.imgmsg_to_cv2(data)
        self.depth_data = cv_depth_image

    def depth_cam_info_callback( self,msg):
        self.depth_cam_info = msg

    def rescale(frame, percent=50):
        width  = int(frame.shape[1] * percent/100)
        height = int(frame.shape[0] * percent/100)
        return cv2.resize(frame, (width,height), interpolation = cv2.INTER_AREA)

def main():
    rospy.init_node("aruco_tracking",anonymous=True)
    rospy.sleep(3)

    # Define calibration object to hold and store points
    calibration = cam_calibration.CameraCalibration()

    # Defiine arguments to pass to calibrate() parameters
    directory_path = "/home/drojas/dlo_ws/src/wire_manipulation/vision/resources/calibration/*"
    img_file_prefix = "img_"
    img_format = ".jpg"
    square_size = 0.127 # in meters; each square is 0.5inch
    height = 20-1 # squares high
    width = 20-1 # squares across

    calibration_matrices = calibration.calibrate(directory_path, img_file_prefix, img_format, square_size, height, width)
    tracker = ArucoTracker(calibration_matrices[1], calibration_matrices[2])

    rospy.spin()

if __name__ == '__main__':
    main()
