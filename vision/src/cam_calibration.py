#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np 
import glob

class CameraCalibration(object):
    def __init__(self):
        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

def main():
    rospy.init_node("camera_calibration")
    rospy.sleep(3)
    calibration = CameraCalibration()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shut down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
