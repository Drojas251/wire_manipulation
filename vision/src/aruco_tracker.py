#!/usr/bin/env python3

import cv2
import numpy as np 
import glob

CAMERA_SRC = cv2.VideoCapture(1)

class ArucoTracker:
    def __init__(self):
        pass

    def track(self, matrix_coefficients, distortion_coefficients):
        while True:
            ret, frame = CAMERA_SRC.read()
            # operations on the frame come here
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale

            aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)  # Use 5x5 dictionary to find markers
            parameters = cv2.aruco.DetectorParameters_create()  # Marker detection parameters
            # lists of ids and the corners beloning to each id
            corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, aruco_dict,
                                                                    parameters=parameters,
                                                                    cameraMatrix=matrix_coefficients,
                                                                    distCoeff=distortion_coefficients)
            if np.all(ids is not None):  # If there are markers found by detector
                for i in range(0, len(ids)):  # Iterate in markers
                    # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                            distortion_coefficients)
                    (rvec - tvec).any()  # get rid of that nasty numpy value array error
                    cv2.aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
                    cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  # Draw Axis
            # Display the resulting frame
            cv2.imshow('frame', frame)
            # Wait 3 milisecoonds for an interaction. Check the key and do the corresponding job.
            key = cv2.waitKey(3) & 0xFF
            if key == ord('q'):  # Quit
                break
    

def main():
    pass

if __name__ == '__main__':
    cv2.aruco
    main()
