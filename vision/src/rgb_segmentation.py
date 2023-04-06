#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np 
from geometry_msgs.msg import Pose

class RGBSegmentation(object):
    def __init__(self):
        # Subscribers to Camera
        self.aligned_depth_rgb_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.get_depth_data,queue_size=1)
        self.rgb_img_sub = rospy.Subscriber("/camera/color/image_raw",Image, self.rgb_callback,queue_size=1)
        self.depth_img_camera_info = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info",CameraInfo, self.depth_cam_info_callback,queue_size=1)
        
        # Publishers with segmented image info
        self.image_pub = rospy.Publisher("/rs_segmented_image", Image, queue_size=1)
        self.depth_image_pub = rospy.Publisher("/seg_depth/image_raw", Image, queue_size=1)
        self.depth_img_cam_info_pub = rospy.Publisher("/seg_depth/camera_info", CameraInfo, queue_size=1)
        
        # Image member variables
        self.bridge_object = CvBridge()
        self.depth_data = []
        self.depth_cam_info = CameraInfo()
        self.seg_depth_img = Image()

    def rgb_callback(self,data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        rospy.sleep(0.01)

        # Segment RGB by Coloe
        lower_color = np.array([ 124, 72, 47])
        upper_color = np.array([179, 255, 255])
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_color, upper_color)
        new_img = cv2.bitwise_and(cv_image, cv_image, mask = mask )

        # dilation
        kernel = np.ones((5,5), np.uint8)
        img_dilation = cv2.dilate(new_img, kernel, iterations=1)
        img_dilation_gray = cv2.cvtColor(img_dilation,cv2.COLOR_BGR2GRAY)
        
        # find largest contour
        contours, hierch = cv2.findContours(img_dilation_gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        largest_area = sorted(contours, key= cv2.contourArea)
        mask2 = np.zeros(img_dilation_gray.shape, np.uint8)

        filtered_wire = cv2.drawContours(mask2,[largest_area[-1]], 0, (255,255,255,255), -1)

        # erosion
        img_erosion = cv2.erode(filtered_wire, kernel, iterations=2)

        # Get copy of depth image
        depth = self.depth_data
        depth_copy = depth.copy()

        # use segmented RGB image as mask for depth image
        new_depth_img = cv2.bitwise_and(depth_copy, depth_copy, mask = img_erosion )

        # Define Camera info for publish
        cam_info = CameraInfo()
        cam_info.header.stamp = rospy.Time.now()
        cam_info.header.frame_id = "camera_color_optical_frame"
        cam_info.height = 720
        cam_info.width = 1280
        cam_info.distortion_model = "plumb_bob"
        cam_info.D = self.depth_cam_info.D
        cam_info.K = self.depth_cam_info.K
        cam_info.R = self.depth_cam_info.R
        cam_info.P = self.depth_cam_info.P
        cam_info.binning_x = 0
        cam_info.binning_y = 0
        cam_info.roi = self.depth_cam_info.roi

        # Segmented RGB Image
        segmented_img = self.bridge_object.cv2_to_imgmsg(new_img,"passthrough")
        segmented_img.header.frame_id = "camera_color_optical_frame"

        # Segmneted Depth Image
        self.seg_depth_img = self.bridge_object.cv2_to_imgmsg(new_depth_img)
        self.seg_depth_img.header.stamp = cam_info.header.stamp
        self.seg_depth_img.header.frame_id = "camera_color_optical_frame"
    
        # Publish
        self.image_pub.publish(segmented_img)
        self.depth_image_pub.publish(self.seg_depth_img)
        self.depth_img_cam_info_pub.publish(cam_info)

    def get_depth_data(self,data):
        cv_depth_image = self.bridge_object.imgmsg_to_cv2(data)
        self.depth_data = cv_depth_image

    def depth_cam_info_callback( self,msg):
        self.depth_cam_info = msg


def main():
    rospy.init_node("seg_node",anonymous=True)
    rospy.sleep(3)
    seg_object = RGBSegmentation()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shut down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
