#!/usr/bin/env python3
import rospy

# tf2 and Transformations
# import tf2_ros
# import tf_conversions
# from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo

# Camera capture
from cv_bridge import CvBridge,CvBridgeError
from collections import defaultdict

import cv2
import numpy as np 
# import cam_calibration

class EnvironmentGrid:
    def __init__(self):
        # Subscribers to Camera
        self.rgb_img_sub = rospy.Subscriber("/mounted_cam/camera/color/image_raw", Image, self.grid_callback ,queue_size=1)
        
        # Image member variables
        self.bridge_object = CvBridge()
        self.seg_depth_img = Image()

    def grid_callback(self, data):
        try:
            frame = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        rospy.sleep(0.01)

        frame = self.draw_grid(frame, (10, 8))
        # Display the resulting frame
        cv2.imshow('frame', frame) 
        cv2.waitKey(1)

    def draw_grid(self, img, grid_shape, color=(0, 0, 255), thickness=1):
        h, w, _ = img.shape
        rows, cols = grid_shape
        dy, dx = h / rows, w / cols

        # draw vertical lines
        for x in np.linspace(start=dx, stop=w-dx, num=cols-1):
            x = int(round(x))
            cv2.line(img, (x, 0), (x, h), color=color, thickness=thickness)

        # draw horizontal lines
        for y in np.linspace(start=dy, stop=h-dy, num=rows-1):
            y = int(round(y))
            cv2.line(img, (0, y), (w, y), color=color, thickness=thickness)

        return img



def main():
    rospy.init_node("environment_grid",anonymous=True)
    rospy.sleep(3)

    # Define object to hold grid of environment
    grid = EnvironmentGrid()

    rospy.spin()

if __name__ == '__main__':
    main()
