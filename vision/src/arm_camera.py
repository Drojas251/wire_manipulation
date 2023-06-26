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

CAMERA_SRC = cv2.VideoCapture(6) # Depth cam device index 4; use when running without ROS

class ArmCamera:
    def __init__(self, matrix_coefficients, distortion_coefficients):
        pass

def main():
    rospy.init_node("arm_camera",anonymous=True)
    rospy.sleep(3)

    while(True):
      
        # Capture the video frame
        # by frame
        ret, frame = CAMERA_SRC.read()
    
        # Display the resulting frame
        cv2.imshow('frame', frame)
        
        # the 'q' button is set as the
        # quitting button you may use any
        # desired button of your choice
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # After the loop release the cap object
    CAMERA_SRC.release()
    # Destroy all the windows
    cv2.destroyAllWindows()

    rospy.spin()

if __name__ == '__main__':
    main()
