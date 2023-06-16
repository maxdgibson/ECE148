"""
This script combines the Arcuo marker detection with PyVesc code
to implement a type of autonomous parking based on the coordinates
and rotations of the marker

Filename: Aruco_VESC_Final.py
Created: 06/14/2023
Author: Artyom M.
"""

import cv2 as cv
from cv2 import aruco
import numpy as np
import depthai as dai
import time
 
class VESC:
    def __init__(self, serial_port, percent=.2, has_sensor=False, start_heartbeat=True, baudrate=115200, timeout=0.05, steering_scale = 1.0, steering_offset = 0.0 ):
 
        try:
            import pyvesc
        except Exception as err:
            print("\n\n\n\n", err, "\n")
            print("please use the following command to import pyvesc so that you can also set")
            print("the servo position:")
            print("pip install git+https://github.com/LiamBindle/PyVESC.git@master")
            print("\n\n\n")
            time.sleep(1)
            raise
 
        assert percent <= 1 and percent >= -1,'\n\nOnly percentages are allowed for MAX_VESC_SPEED (we recommend a value of about .2) (negative values flip direction of motor)'
        self.steering_scale = steering_scale
        self.steering_offset = steering_offset
        self.percent = percent
 
        try:
            self.v = pyvesc.VESC(serial_port, has_sensor, start_heartbeat, baudrate, timeout)
        except Exception as err:
            print("\n\n\n\n", err)
            print("\n\n fix permission errors")
            time.sleep(1)
            raise
    ''' This particular file only shows the implementation involving the steering and throttle control from VESC
     `VESC.py of the PYVesc repository can be referred for additional functionalities''' 
 
    def run(self, angle, throttle):
        '''Input angle (0-1) and throttle (0 - 1)
            Steering center is at an angle of 0.5 for ECE/MAE 148. The offset can be adjusted using steering offset
            attribute'''
 
        self.v.set_servo((angle * self.steering_scale) + self.steering_offset)
        self.v.set_duty_cycle(throttle*self.percent)
 
VESC_module = VESC('/dev/ttyACM0')
 
calib_data_path = "../calib_data/MultiMatrix.npz"
 
calib_data = np.load(calib_data_path)
print(calib_data.files)
 
cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]
 
MARKER_SIZE = 6  # centimeters
 
marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
 
param_markers = aruco.DetectorParameters_create()
 
arucoList = []
 
# Create pipeline
pipeline = dai.Pipeline()
 
# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
camRgb.initialControl.setManualFocus(145)
camRgb.setFps(40)
camRgb.setPreviewKeepAspectRatio(False)
 
rgbOut = pipeline.create(dai.node.XLinkOut)
rgbOut.setStreamName("rgb")
camRgb.preview.link(rgbOut.input)
 
markerOut = pipeline.create(dai.node.XLinkOut)
markerOut.setStreamName("markers")
 
# Connect to a device and start the pipeline
with dai.Device(pipeline) as device:
    rgbQueue = device.getOutputQueue("rgb", maxSize=4, blocking=False)
    markerQueue = device.getOutputQueue("markers", maxSize=4, blocking=False)
 
    while True:
        inRgb = rgbQueue.get()
        frame = inRgb.getCvFrame()
 
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
 
        marker_corners, marker_IDs, reject = cv.aruco.detectMarkers(
            gray_frame, marker_dict, parameters=param_markers
        )
 
        marker_info = [] # Stores marker information
 
        if marker_corners:
            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                marker_corners, MARKER_SIZE, cam_mat, dist_coef
            )
            total_markers = range(0, marker_IDs.size)
            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                cv.polylines(
                    frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
                )
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()
 
                # Since there was a mistake in calculating the distance approach point-outed in the Video Tutorial's comment
                # so I have rectified that mistake, I have tested that out it increases the accuracy overall.
                # Calculating the distance
                distance = np.sqrt(
                    tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                )
 
                # Extract rotation angles
                r_x = round(rVec[i][0][0], 1)
                r_y = round(rVec[i][0][1], 1)
                r_z = round(rVec[i][0][2], 1)
 
                # Add marker info to the list
                marker_info.append({
                    "id": ids[0],
                    "x": round(tVec[i][0][0], 1),
                    "y": round(tVec[i][0][1], 1),
                    "z": round(tVec[i][0][2], 1),
                    "r_x": r_x,
                    "r_y": r_y,
                    "r_z": r_z
                })
 
                # Draw the pose of the marker
                point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
                cv.putText(
                    frame,
                    f"id: {ids[0]} Dist: {round(distance, 2)}",
                    top_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.3,
                    (0, 0, 255),
                    2,
                    cv.LINE_AA,
                )
                cv.putText(
                    frame,
                    f"x:{round(tVec[i][0][0], 1)} y: {round(tVec[i][0][1], 1)} ",
                    bottom_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.0,
                    (0, 0, 255),
                    2,
                    cv.LINE_AA,
                )
        cv.imshow("frame", frame)
        key = cv.waitKey(1)
        if key == ord("q"):
            break
        # Add all detected aruco tags to arucoList
        for marker in marker_info:
            arucoElement = [marker["id"], marker["x"], marker["y"], marker["z"], marker["r_y"]]
            arucoList.append(arucoElement)
        # Calculate and move the VESC if any aruco marker is detected
        if arucoList != []:        
            throttle = 0.15
            angle = (np.arctan2(arucoList[0][1], arucoList[0][3]) + np.pi) / (2 * np.pi)
            distance = arucoList[0][3]
            # If the marker is next to the camera (i.e. the car parked in front) stop motors
            if distance < 40.0:
                throttle = 0.0
            VESC_module.run(angle, throttle)
            print(arucoList, angle, distance) # Real-time display of data
        else:
            VESC_module.run(0.5, 0.0) # No aruco so don't move and face stright
        arucoList = [] # Empty arucoList after every frame
 
    cv.destroyAllWindows()
