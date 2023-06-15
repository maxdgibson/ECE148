#!/usr/bin/env python
# coding: utf-8

# In[1]:


#https://roboticsbackend.com/write-minimal-ros2-python-node/ 
#https://github.com/HotBlackRobotics/hotblackrobotics.github.io/blob/master/en/blog/_posts/2018-01-29-action-client-py.md
#http://wiki.ros.org/move_base#move_base-1


#!/usr/bin/env python
# ^^This has to do with environment I think, I don't know if this is correct tho -Gus



##########################################################################################################################
    #IMPORTS, PREPARATION
#Rospy!
import rospy

import cv2 as cv
from cv2 import aruco
import numpy as np

# MathStuff!
import numpy as np
from tf.transformations import euler_from_quaternion
import math

# Brings in the SimpleActionClient
import actionlib

# Brings in the .action file and messages used by the move base action, different stuff for msg's
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from scipy.spatial import distance

#Initial Values/Declarations
roll = pitch = yaw = 0.0
GoalNode=[0,0,0,0,0]

# Aruco Setsup

# from depthai_library import DepthAI
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

cap = cv.VideoCapture(0) #outdated code (used with laptop camera)
#depthai = DepthAI()
#depthai.init_device()

##########################################################################################################################
#Utility Function Definitions
##########################################################################################################################
#!/usr/bin/env python

#Code to pull rotation from Odometry, based on https://answers.ros.org/question/290534/rotate-a-desired-degree-using-feedback-from-odometry/
def get_yaw (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return yaw
    
    ###???????? Don't know if this works or not????#########
def get_pos (msg):
    global GPosX, GPosY, GposZ
    orientation_t = msg.pose.pose.position
    (GPosX, GPosY, GposZ) = orientation_t
    return (GPosX, GPosY, GposZ)
    
# Camera Z+ = facing out from camera
# Camera X+ = to the right of camera

# camera = camera offset 1x5
# nodes = sorted arucos 5xL
# WARNING, sorted arucos already have same IDs
# 1s or 0s for ID
def closest_node(camera, nodes):
    closest_index = distance.cdist([camera], nodes).argmin()
    return nodes[closest_index]

# PRIMARY FUNCTION DEFS

## the code for sorting arucos and prioritizing one. to be made into a function that we can use int he node 'while not rospy is shutdown' while loop, so if the node is active it's taking the stuff from the camera and working with it.
def sorting_prioritization(ArucosNonNP)
# Camera Offset from the center of bot in cm
    CameraOffset = [0,0,0,-20,0]
    # Pseudo code for retrieval, prioritization, preparation
    # Retrieve Aruco Marker Position Array
    Arucos = np.matrix(ArucosNonNP)
    
    # Filtered Arucos
    # Array that only contains one ID
    FilteredArucos = np.zeros_like(Arucos)

    # I'm assuming the array is such that each row is a different marker
    #Column 1 is ID (0 is waypoint, 1 is parking)
    #2 is X (relative to camera view, center = 0 right = x+)
    #3 is Y (relative to camera view, center = 0 right = y+)
    #4 is Z (Relaitve to Camera view, center=0 right= z+)
    #5 is Yaw ( Parallel = 0, clockwise = theta+)
    
    # Distance is presumed to be in cm, 
    
    #https://stackoverflow.com/questions/58079075/numpy-select-rows-based-on-condition
    #Observed=np.size(FilteredArucos, 0)
    if 1 in Arucos[:, 0]:
        mask = (Arucos[:, 0] == 1)
        FilteredArucos = Arucos[mask, :]
    else:
        FilteredArucos = Arucos
        
    # [0,0,0] is camera offset
    # The argmin in closest_node will take the first
    ChosenNode = closest_node(CameraOffset, FilteredArucos)
    
    # OFFset position of SA by the camera position relative to center of bot
    # find rotation of aruco relative to global axis using known angle and measured angle from camera (Gtheta)
    # Use the below commands to write the Gx, Gy, Gtheta as the new movement goal
    ChosenNode = ChosenNode + CameraOffset
    
    ChosenNode[1] = ChosenNode[1] * 0.01 #Converting from Cm to M
    ChosenNode[2] = ChosenNode[2] * 0.01 #Converting from Cm to M
    ChosenNode[3] = ChosenNode[3] * 0.01 #Converting from Cm to M
    # Rotations assumed to be all in RADIANS
    # Yaw=0 means bot is aligned facing the X+ direction.
    # Camera Z+ => Global X+
    # Camera X+ => Global Y-
    BOT_YAW = get_yaw #Radians from Odometry
    BOT_POS = get_pos
    GoalNode[0]=0
    GoalNode[1]= (ChosenNode[3]*np.cos(BOT_YAW) - ChosenNode[1]*np.sin(BOT_YAW)) + (BOT_POS[1]) # Rotation of coordinates to align with Global axis, add bot pos X to get global position of goal
    GoalNode[2]= (ChosenNode[1]*np.cos(BOT_YAW) + ChosenNode[3]*np.sin(BOT_YAW)) + (BOT_POS[2]) # Rotation of coordinates to align with Global axis, add bot pos Y to get global position of goal
    GoalNode[3]= (ChosenNode[3] + BOT_POS[2]) # Z axis, doesn't change. Add Z of bot just in case
    GoalNode[4]= ChosenNode[4] + BOT_YAW # Combines bot yaw and observed aruco yaw to get global yaw of bot


def movebase_client(GoalNode): #Takes coordinate GoalNode (from the prioritization function) and sends it to movebase
    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()
        
    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Move to GoalNode Position relative to map frame
    goal.target_pose.pose.position.x = GoalNode[1]
    goal.target_pose.pose.position.y = GoalNode[2]
    
    # Goal rotation of the mobile base frame w.r.t. map frame (Aruco Rotation)
    goal.target_pose.pose.orientation.w = GoalNode[4]

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   
    # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        # Result of executing the action
        return client.get_result()   


#################################################################################### NODE STUFF
# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

rospy.init_node('aruco_to_command') ##DEFINES NODE NAME

odoyaw = rospy.Subscriber ('/odom', Odometry, get_yaw)
odopos = rospy.Subscriber ('/odom', Odometry, get_pos)
pubvel = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #What is Twist????
# 
command= Twist() #???????????????????????

r = rospy.Rate(10)
Parki=0
while not rospy.is_shutdown():  #This while loop should run continously while the node is on https://www.youtube.com/watch?v=jWtkzDbez9M
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        if Parki==0:
            Arucos=Aruco_Distance()
            GoalNodi= sorting_prioritization(Arucos)
            result = movebase_client(GoalNodi)
            if result:
                if GoalNodi[0]==1:
                    rospy.loginfo("Parked!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
    pubvel.publish(command)
    r.sleep()


# In[ ]:
