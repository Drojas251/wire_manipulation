#!/usr/bin/env python3
# import rospy
# from sensor_msgs.msg import Image, CameraInfo
# from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np 
import glob

class CameraCalibration(object):
    def __init__(self):
        # Termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # Define lists to store points from images
        self.points_3d = [] # 3d points in real world space
        self.points_2d = [] # 2d points in image space

    def calibrate(self, directory_path, img_file_prefix, img_format, square_size, height, width):
        # Define board matrix mapping calibration sheet
        object_point = np.zeros((height * width, 3), np.float32)
        object_point[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)
        # Represent square_size in meters, width/height are number of squares each way
        object_point *= square_size

        # Define images and loop to find corners on board
        images = glob.glob(directory_path + '/' + img_file_prefix + "*."  + img_format)
        for i in images:
            image = cv2.imread(i)
            gray  = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) # Process for gray because B/W board
            # Finding corners
            ret, corners = cv2.findChessboardCorners(gray, (width, height), None)
            # If corners found, add and store to our result
            if ret:
                self.points_3d.append(object_point)
                processed_corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), self.criteria) # might need to adjust winSize
                self.points_2d.append(processed_corners)

                # Draw and display the found corners for verification
                preview = cv2.drawChessboardCorners(image, (width, height), processed_corners, ret)

        # Finally, calibrate and return matrices of image
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.points_3d, self.points_2d, gray.shape[::-1], None, None)
        return [ret, mtx, dist, rvecs, tvecs]
    
    ## Helpful functions for image calibration
    def save_coefficients(mtx, dist, path):
        """
        Save the camera matrix and the distortion coefficients to given path/file.
        """
        cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
        cv_file.write("K", mtx)
        cv_file.write("D", dist)
        cv_file.release()
    
    def load_coefficients(path):
        """
        Loads camera matrix and distortion coefficients.
        """
        # FILE_STORAGE_READ
        cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

        camera_matrix = cv_file.getNode("K").mat()
        dist_matrix = cv_file.getNode("D").mat()

        cv_file.release()
        return [camera_matrix, dist_matrix]
    

def main():
    # rospy.init_node("camera_calibration")
    # rospy.sleep(3)
    calibration = CameraCalibration()
    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print("shut down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
