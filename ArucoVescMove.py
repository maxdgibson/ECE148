import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import cv2 as cv
from cv2 import aruco
import numpy as np
import time

# Define the VESC class for motor control
class VESC:
    def __init__(self, serial_port, percent=.2, has_sensor=False, start_heartbeat=True, baudrate=115200, timeout=0.05, steering_scale=1.0, steering_offset=0.0):
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

        assert percent <= 1 and percent >= -1, '\n\nOnly percentages are allowed for MAX_VESC_SPEED (we recommend a value of about .2) (negative values flip direction of motor)'
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

    def run(self, angle, throttle):
        self.v.set_servo((angle * self.steering_scale) + self.steering_offset)
        self.v.set_duty_cycle(throttle * self.percent)

# Initialize the ROS node
rospy.init_node('movebase_aruco_example', anonymous=False)

# Create a client for the MoveBase action server
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()

# ArUco marker detection parameters and settings
calib_data_path = "../calib_data/MultiMatrix.npz"
MARKER_SIZE = 8  # centimeters
marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters_create()

# VESC motor control parameters and settings
vesc_serial_port = '/dev/ttyACM0'
vesc_percent = 0.2  # Adjust as needed
vesc = VESC(vesc_serial_port, percent=vesc_percent)

def detect_aruco_markers(frame):
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, _ = aruco.detectMarkers(gray_frame, marker_dict, parameters=param_markers)
    marker_info = []
    if marker_corners:
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corners, MARKER_SIZE, cam_mat, dist_coef)
        total_markers = range(0, marker_IDs.size)
        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            top_right = corners[0].ravel()
            top_left = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()

            # Calculating the distance
            distance = np.sqrt(tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2)

            marker_info.append({
                'id': ids[0],
                'distance': round(distance, 2),
                'x': round(tVec[i][0][0], 1),
                'y': round(tVec[i][0][1], 1)
            })

    return marker_info

cap = cv.VideoCapture(0)

on_correct_track = False

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        break

    marker_info = detect_aruco_markers(frame)

    for info in marker_info:
        print(f"Marker ID: {info['id']}")
        print(f"Distance: {info['distance']}")
        print(f"X: {info['x']}")
        print(f"Y: {info['y']}")
        print()

        if info['id'] == 0:
            on_correct_track = True
        elif info['id'] == 1:
            on_correct_track = False
            # Perform the actions to move to the goal associated with Marker ID 1
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            #goal.target_pose.pose.position.x = desired_x
            #goal.target_pose.pose.position.y = desired_y
            #goal.target_pose.pose.orientation.w = desired_orientation_w
            client.send_goal(goal)
            client.wait_for_result()
            # After reaching the goal, idle the VESC motors
            vesc.run(0, 0)

    if on_correct_track:
        # Perform actions for staying on the correct track
        # Adjust motor control based on your requirements
        vesc.run(0.75, 0.2)
    else:
        # Perform actions for being off the correct track
        # Adjust motor control based on your requirements
        vesc.run(0, 0)

    cv.imshow("frame", frame)
    key = cv.waitKey(1)
    if key == ord("q"):
        break

cap.release()
cv.destroyAllWindows()
