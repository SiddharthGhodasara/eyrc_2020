#! /usr/bin/env python

#Importing Libraries
import os
import tf
import cv2
import time
import rospy
import rospkg
import tf2_ros
import numpy as np
import geometry_msgs.msg
import tf2_geometry_msgs
from PIL import Image as img
from std_msgs.msg import String
from object_msgs.msg import ObjectPose
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel

import sys
import copy
import rospy
import tf
import moveit_commander 
import moveit_msgs.msg
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal

#Getting the parameters from launch file
image_topic = '/camera/color/image_raw2'
camera_topic = '/camera/color/camera_info2'
depth_topic = '/camera/depth/image_raw2'
camera_frame = 'camera_link2'

#Defining the bridge
bridge = CvBridge()

pose_dir =  dict()

#Declaring the publisher
pub = None
#Defining camera info global varibale
camera_info = None
cam_model = None
#Storing the depth
depth = None

arm_group = None
hand_group = None
robot = None

#Tranform Listener
tf_listener = None
#Transformations
tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length

#Loading the labels
rospack = rospkg.RosPack()
package_path = rospack.get_path('ebot_mani')
labelsPath = os.path.sep.join([package_path, 'yolo', 'obj.names'])
LABELS = open(labelsPath).read().strip().split("\n")

#Derive the paths to the YOLO weights and model configuration
weightsPath = os.path.sep.join([package_path, 'yolo', "yolov3-tiny-obj_1000.weights"])
configPath = os.path.sep.join([package_path, 'yolo', "yolov3-tiny-obj.cfg"])

# load our YOLO object detector trained on custom data
print("[INFO] loading YOLO from disk...")
net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)


def transform_frames(x,y,z,frame):
    #Making Point Stamp Message
    grasp_pose = geometry_msgs.msg.PoseStamped()
    grasp_pose.header.frame_id = camera_frame
    grasp_pose.pose.position.x =  x
    grasp_pose.pose.position.y =  y
    grasp_pose.pose.position.z =  z

    #Transforming
    target_frame = frame
    source_frame = camera_frame
    transform = tf_buffer.lookup_transform(target_frame,
                                           source_frame, #source frame
                                           rospy.Time(0), #get the tf at first available time
                                           rospy.Duration(1.0)) #wait for 1 second

    #Applying the transform
    pose_transformed = tf2_geometry_msgs.do_transform_pose(grasp_pose, transform)
    pose_transformed.pose.orientation.x = 0
    pose_transformed.pose.orientation.y = 0
    pose_transformed.pose.orientation.z = 0
    pose_transformed.pose.orientation.w = 1
    #Returning the transform coordinates
    return pose_transformed

#Function for converting coordinates to base link frame
def uv_to_xyz(cx, cy):
    #Converting to XYZ coordinates
    (x, y, z) = cam_model.projectPixelTo3dRay((cx, cy))
    #Normalising
    x = x/z
    y = y/z
    z = z/z

    #Getting the depth at given coordinates
    depth = rospy.wait_for_message(depth_topic, Image)
    print(type(depth))
    depth_img = img.frombytes("F", (depth.width, depth.height), depth.data)
    lookup = depth_img.load()
    d = lookup[cx, cy]

    #Modifying the coordinates
    x *= d
    y *= d
    z *= d
    pose = transform_frames(x,y,z,"odom")
    tmp = transform_frames(x,y,z,"ebot_base")
    print(pose)
    return pose


#Prediction Function
def prediction(image):
    global pose_dir
    #Getting the image dimensions
    (H, W) = image.shape[:2]

    # determine only the *output* layer names that we need from YOLO
    ln = net.getLayerNames()
    ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    #Creating Blob from images
    blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
    #Feeding the blob as input
    net.setInput(blob)
    #Forward pass
    layerOutputs = net.forward(ln)

    #Initializing lists of Detected Bounding Boxes, Confidences, and Class IDs
    boxes = []
    confidences = []
    classIDs = []

    #Looping over each of the layer outputs
    for output in layerOutputs:
    	#Looping over each of the detections
    	for detection in output:
    		#Extracting the class ID and confidence
    		scores = detection[5:]
    		classID = np.argmax(scores)
    		confidence = scores[classID]

            #Filtering out weak predictions
    		if confidence > 0.5:
    			#Scale the Bounding Box Coordinates
    			box = detection[0:4] * np.array([W, H, W, H])
    			(centerX, centerY, width, height) = box.astype("int")
                # use the center (x, y)-coordinates to derive the top and
    			# and left corner of the bounding box
    			x = int(centerX - (width / 2))
    			y = int(centerY - (height / 2))
                #Updating the Lists
    			boxes.append([x, y, int(width), int(height)])
    			confidences.append(float(confidence))
    			classIDs.append(classID)

    #Applying non-max Suppression
    idxs = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.3)

    #Ensuring at least one detection exists
    if len(idxs) > 0:
    	#Looping over the indexes
        print(len(idxs))
        label_str = ""
    	for i in idxs.flatten():
            #Extracting the bounding box coordinates
            (x, y) = (boxes[i][0], boxes[i][1])
            (w, h) = (boxes[i][2], boxes[i][3])
            #Getting the labels
            label = LABELS[classIDs[i]]
            print(label)

            #Getting center coordinates
            cx = x + (w/2)
            cy = y + (h/2)
            
            #Converting the center coordinates to base link frame
            xy_pose = ObjectPose()
            xy_pose.pose = uv_to_xyz(cx, cy)

            xy_pose.name = label
            # print(xy_pose)

            pose_dir[label]= xy_pose
            
            #Publishing
            pub.publish(xy_pose)
            print("Published")



# Function to move the arm 
# Arguments:- 6D Pose of the location (position and orientation(in qauternion))
#           - Gripper's closing value
#           - Joint value of the robot (waist joint of the robot) which is unique and thus is given individually (we have used joint angles for diversify
#             from the conventional 6D pose to specifiy goal location. The same output was found when feeding moveit with 6D poses.)
'''
def move_arm(pose, gripper_v):
    global arm_group
    global hand_group
    
    # STEP 1 
    # Move the robot to hover above the object.
    # Giving the robot a 6D pose on the object location.
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = pose.pose.pose.position.x
    pose_target.position.y = pose.pose.pose.position.y - 0.1
    pose_target.position.z = 0.93

    pose_target.orientation.x = -0.0267950343469
    pose_target.orientation.y = -0.883912377391
    pose_target.orientation.z = -0.466875953503
    pose_target.orientation.w = 0.00278913429132

    # Specifying the target pose to the planning group of the robot.
    arm_group.set_pose_target(pose_target)

    # Giving the robot a maximum planning time 20 sec to plan its path.
    arm_group.set_planning_time(40)

    # Planning a path to reach the location
    plan1 = arm_group.plan()

    # Executing the path
    arm_group.go(wait=True)


    # STEP 2 waypoints = []
    wpose = arm_group.get_current_pose().pose

    # Move up (z) by 20 cm
    wpose.position.z +=  0.2  

    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = arm_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0) 
    arm_group.execute(plan,wait=True)
    # The robot has to go down by a specific amount to ensure that the object is in between the gripper fingers.
    # Empty list for waypoints to be followed.
    waypoints = []

    # Getting the current pose of the robot, essential as the robot would then iterate cartesian path from the that pose.
    wpose = arm_group.get_current_pose().pose

    # Move sideways (y) by 10cm
    wpose.position.y +=  0.1  
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm. Thus we specify eef_step as 0.01. 
    # We disable the jump threshold by setting it to 0.0.
    (plan, fraction) = arm_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0) 

    # Sending the plan to the robot for execution.
    arm_group.execute(plan,wait=True)


    # STEP 3
    # Closing the gripper by specific amount 
    # Obtaining the current value of the gripper 
    group_variable_values = hand_group.get_current_joint_values()

    # Moving the gripper to a specific joint value
    group_variable_values[0] = gripper_v

    hand_group.set_joint_value_target(group_variable_values)

    # Sending the values to the robot for execution
    hand_group.go(wait=True)


    # STEP 4
    # The robot then proceeds to move up by a specific amount before moving towards goal location.
    # This cartesian path code is as explained above.
    waypoints = []

    wpose = arm_group.get_current_pose().pose

    # Move up (z) by 20 cm
    wpose.position.z +=  0.2  

    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = arm_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0) 
    arm_group.execute(plan,wait=True)


    # STEP 5
    # Move the robot to drop location 
    # Obtaining the robot's current joint values.
    arm_group.set_named_target("drop_pose")

    # Sending the joint set to the arm to execute a path to that specific location.
    arm_group.go(wait=True)



    # STEP 6
    # Opening the gripper to drop the object.
    hand_group.set_named_target("gripper_open")
    hand_group.go(wait=True)

    waypoints = []
    wpose = arm_group.get_current_pose().pose

    # Move up (z) by 20 cm
    wpose.position.z +=  0.2  

    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = arm_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0) 
    arm_group.execute(plan,wait=True)
'''

#Image Callback Function
def callback(data):
    try:
        #Converting Sensor Image to cv2 Image
        cv_img = bridge.imgmsg_to_cv2(data, 'bgr8')
        #Calling the prediction function
        prediction(cv_img)
    #Handling exceptions
    except CvBridgeError as e:
        print(e)

#Main Function
def main():
    #Defining global variable usage
    global pub
    global cam_model
    global tf_listener

    global arm_group
    global hand_group
    global robot

    # Initializing moveit_commander 
    moveit_commander.roscpp_initialize(sys.argv)

    # Initializing rospy node
    rospy.init_node('move_python_node',anonymous=True)

    # Instantiating a RobotCommander object. This provides information on robot's current joint states.
    robot = moveit_commander.RobotCommander() 

    # Instantiating a PlanningScene object(provides info on obtaining and updating robot's understanding of the world)
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiating a MoveGroupCommander object. This object is an interface to a planning group.
    arm_group = moveit_commander.MoveGroupCommander("arm")
    hand_group= moveit_commander.MoveGroupCommander("gripper")

    # Create a DisplayTrajectory ROS publisher (used to display trajectories in Rviz)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path' , moveit_msgs.msg.DisplayTrajectory,queue_size=20)

    arm_group.set_named_target("init_pose")

    # Sending the arm to execute a path to that specific location.
    arm_group.go(wait=True)

    #Getting camera info
    camera_info = rospy.wait_for_message(camera_topic, CameraInfo)
    #Setting up Camera Model
    cam_model = PinholeCameraModel()
    cam_model.fromCameraInfo(camera_info)

    #Initialising Transformations
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    #Initialising the publisher
    pub = rospy.Publisher('/detection_info', ObjectPose, queue_size=10)
    #Subscribinig to image topic

    print("Waiting for image")
    sub = rospy.wait_for_message(image_topic, Image)
    print("Obtained image")
    callback(sub)
    print(pose_dir)
	'''
    move_arm(pose_dir["coke_can"], 0.25)
    rospy.sleep(1)
    move_arm(pose_dir["battery"], 0.3)
    rospy.sleep(1)
    move_arm(pose_dir["glue"], 0.25)
	'''
    

#Calling the main thread
if __name__ == "__main__":
    main()
