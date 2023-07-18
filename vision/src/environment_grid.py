#!/usr/bin/env python3
import rospy

# tf2 and Transformations
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
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

        """
        https://automaticaddison.com/how-to-convert-camera-pixels-to-robot-base-frame-coordinates/
        https://stackoverflow.com/questions/23606600/how-to-determinate-object-position-with-opencv-in-the-world-coordinate
        """

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

def environment_origin(source: str, pos_adj, ori_adj) -> None:
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = source
    t.child_frame_id = "environment_origin".format()

    t.transform.translation.x = ori_adj[0]
    t.transform.translation.y = ori_adj[1]
    t.transform.translation.z = ori_adj[2]
    
    q = quaternion_from_euler(pos_adj[0], pos_adj[1], pos_adj[2]) # pos_adj
    # q = quaternion_from_euler(-math.pi/2,math.pi/2,0) # match rotation of bot grippers

    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

def main():
    rospy.init_node("environment_grid",anonymous=True)
    rospy.sleep(3)

    # Define object to hold grid of environment
    grid = EnvironmentGrid()
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        environment_origin("camera_link", [0, 0, 0], [.75, 0, 0])
        # USE THIS FRAME AS GRID CORNER, ITERATE FROM THERE??

    rospy.spin()

if __name__ == '__main__':
    main()
