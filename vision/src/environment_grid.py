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

        # Grid variables
        self.grid = None
        self.grid_x = None
        self.grid_y = None
        self.midpoints = None

    def grid_callback(self, data):
        try:
            frame = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        rospy.sleep(0.01)

        if not self.grid:
            self.grid,self.midpoints = self.build_grid(frame, (12,20))

        frame = self.draw_grid(frame)
        frame = self.draw_pts(frame)
        # Display the resulting frame
        cv2.imshow('frame', frame) 
        cv2.waitKey(1)

    def build_grid(self, img, grid_shape):
        h, w, _ = img.shape
        rows, cols = grid_shape
        dy, dx = h / rows, w / cols

        # Trim grid to working environment
        dropped_col = 4
        dropped_col_end = 2
        dropped_row = 2
        self.grid_x = np.linspace(start=dx, stop=w-dx, num=cols-1)[dropped_col:-dropped_col_end]
        self.grid_y = np.linspace(start=dy, stop=h-dy, num=rows-1)[dropped_row:]

        grid = []
        midpoints = []
        # Calculate grid x,y intersections
        for x in self.grid_x:
            for y in self.grid_y:
                grid.append((x,y))
        # Calculate grid center points
        for x in self.grid_x[:-1]:
            for y in self.grid_y[:-1]:
                midpoints.append((x+dx/2, y+dy/2))

        return grid, midpoints
    
    def draw_pts(self, img):
        # Draw grid points
        for coord in self.grid:
            img = cv2.circle(img, (int(coord[0]), int(coord[1])), radius=4, color=(0, 255, 0), thickness=-1)
        # Draw midpoints
        for coord in self.midpoints:
            img = cv2.circle(img, (int(coord[0]), int(coord[1])), radius=4, color=(0, 255, 0), thickness=-1)

        
        return img

    def draw_grid(self, img, color=(0, 0, 255), thickness=1):
        h, w, _ = img.shape
        # draw vertical lines
        for x in self.grid_x:
            x = int(round(x))
            cv2.line(img, (x, 0), (x, h), color=color, thickness=thickness)

        # draw horizontal lines
        for y in self.grid_y:
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
