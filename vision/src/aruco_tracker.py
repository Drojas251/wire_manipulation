#!/usr/bin/env python3

import cv2
import numpy as np 
import glob
import cam_calibration

CAMERA_SRC = cv2.VideoCapture(4) # Depth cam device index 4
print(CAMERA_SRC)

class ArucoTracker:
    def __init__(self):
        pass

    def track(self, matrix_coefficients, distortion_coefficients):
        while True:
            ret, frame = CAMERA_SRC.read()
            # operations on the frame come here
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
            # cv2.imshow('frame',frame)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break

            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
            parameters =  cv2.aruco.DetectorParameters()
            # lists of ids and the corners beloning to each id
            corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, aruco_dict,
                                                                    parameters=parameters)
            
            
            if np.all(ids is not None):  # If there are markers found by detector
                for i in range(0, len(ids)):  # Iterate in markers
                    # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.127, matrix_coefficients,
                                                                            distortion_coefficients)
                    (rvec - tvec).any()  # get rid of that nasty numpy value array error
                    cv2.aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
                    # cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  # Draw Axis
                    cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, .2) 

            # Display the resulting frame
            cv2.imshow('frame', frame)
            # Wait 3 milisecoonds for an interaction. Check the key and do the corresponding job.
            key = cv2.waitKey(3) & 0xFF
            if key == ord('q'):  # Quit
                break
    

def main():
    # Define calibration object to hold and store points
    calibration = cam_calibration.CameraCalibration()

    # Defiine arguments to pass to calibrate() parameters
    directory_path = "vision/resources/calibration/*"
    img_file_prefix = "img_"
    img_format = ".jpg"
    square_size = 0.127 # in meters; each square is 0.5inch
    height = 20-1 # squares high
    width = 20-1 # squares across

    calibration_matrices = calibration.calibrate(directory_path, img_file_prefix, img_format, square_size, height, width)

    tracker = ArucoTracker()
    tracker.track(calibration_matrices[1], calibration_matrices[2])

if __name__ == '__main__':
    cv2.aruco
    main()
