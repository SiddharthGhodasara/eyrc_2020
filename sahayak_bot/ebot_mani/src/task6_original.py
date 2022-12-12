#!/usr/bin/env python

'''
TEAM ID: SB#337

TEAM MEMBERS: SIDDHARTH GHODASARA (TEAM LEADER)
			  GAURAV SETHIA
			  HARISH GURAGOL
			  VINEET RANJAN
		  
CODE DESCRIPTION: -> This code is responsible for completing Task 6 Original, ie. Transporting 2 objects (battery and coke) to their respective locations (Dropboxes) bu using the navigation and moveit, with object poses being published by Yolo.

				  -> The following steps are taken to ensure the robot performs this task:
					 
					 	1. The robot navigates to the room where it has to pick required object from
						2. The UR5 arm moves to pose from where the objects are visible to the wrist mounted real sense camera
						3. YOLO based 3D based pose estimation node gives the 3D pose of the required object
						4. The UR5 arm picks the object.
						5. The robot navigates to the room where it needs to drop the object
						6. UR5 arm plans a path to the dropping location
					 
					 The above steps take place for each object.


ORDER IN WHICH OBJECTS ARE PICKED - 1. Battery from store room
									2. Coke from Pantry
									

SUBSCRIBER: "/camera/color/image_raw2"     MESSAGE TYPE: sensor_msg.Image
		    "/camera/color/camera_info2"   MESSAGE TYPE: sensor_msg.CameraInfo
			"/camera/depth/image_raw2"     MESSAGE TYPE: sensor_msg.Image

MEMBER FUNCTIONS : 

1. get_image: 	 Callback Function for Subcriber of RGB Image
2. uv_to_xyz: 	 Converts 2D image coorindates (u,v) to 3D world coordinates(x, y, z) wrt required frame
3. yolo: 		 Returns the 3D pose of the required object
4. navigate:     Navigate the robot to the given coorindates
5. pick_object:  Responsible for picking up the given object
6. place_object: Responsible for placing the given object
7. pantry: 		 Responsible to searching and picking coke can in pantry


YOLO Weights and Config file can be found at the below mentioned drive link

	LINK - https://drive.google.com/file/d/1M7DvtQuges6xuzAxMU3GZhfSD4uUZJ6U/view?usp=sharing

The folder should be downloaded, unziped and placed in the shayak_bot package without any renaming
'''
# Importing libraries
import sys
import copy
import rospy
import tf
import time
import moveit_commander 
import moveit_msgs.msg
import geometry_msgs.msg
from object_msgs.msg import ObjectPose
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import os
import cv2
import rospkg
import tf2_ros
import numpy as np
import geometry_msgs.msg
import tf2_geometry_msgs
from PIL import Image as img
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
from termcolor import colored


#Defining the bridge
bridge = CvBridge()

# Initializing moveit_commander 
moveit_commander.roscpp_initialize(sys.argv)

# Initializing rospy node
rospy.init_node('task6_original',anonymous=True)

# Instantiating a RobotCommander object. This provides information on robot's current joint states.
robot = moveit_commander.RobotCommander() 

# Instantiating a PlanningScene object(provides info on obtaining and updating robot's understanding of the world)
scene = moveit_commander.PlanningSceneInterface()

# Instantiating a MoveGroupCommander object. This object is an interface to a planning group.
arm_group = moveit_commander.MoveGroupCommander("arm")
hand_group= moveit_commander.MoveGroupCommander("gripper")

# Create a DisplayTrajectory ROS publisher (used to display trajectories in Rviz)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path' , moveit_msgs.msg.DisplayTrajectory,queue_size=20)

# Move the arm to initial known location.
arm_group.set_named_target("init_pose")

# Sending the arm to execute a path to that specific location.
arm_group.go(wait=True)

# Defining Camera Topics
image_topic = '/camera/color/image_raw2'
camera_topic = '/camera/color/camera_info2'
depth_topic = '/camera/depth/image_raw2'
camera_frame = 'camera_rgb_frame2'

#Tranform Listener
tf_listener = None
#Transformations
tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length

#Getting camera info
camera_info = rospy.wait_for_message(camera_topic, CameraInfo)

#Setting up Camera Model
cam_model = PinholeCameraModel()
cam_model.fromCameraInfo(camera_info)

#Dictionary to store pose
pose_dir = {}

#Loading the labels
rospack = rospkg.RosPack()
package_path = rospack.get_path('ebot_mani')
labelsPath = os.path.sep.join([package_path,'yolo', "obj.names"])
LABELS = open(labelsPath).read().strip().split("\n")

#Derive the paths to the YOLO weights and model configuration
weightsPath = os.path.sep.join([package_path,'yolo', "yolov3-tiny-obj_3000.weights"]
configPath =  os.path.sep.join([package_path,'yolo', "yolov3-tiny-obj.cfg"])

#Load our YOLO object detector trained on custom data
print colored("[INFO] loading YOLO from disk...", "blue")
net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)

#Initialising Transformations
tf_listener = tf2_ros.TransformListener(tf_buffer)
image = None

# Callback function for image topic
def get_image(img):
	#Try Except for exception handling
	try:
		#Converting Sensor Image to cv2 Image
		global image
		image = bridge.imgmsg_to_cv2(img, 'bgr8')
	
	except CvBridgeError as e:
		print(e)

# Function to convert 2D image coordinates to 3D coordinates
# Arugments 	- cx, cy - 2D image coordinates
# Return Value 	- PointStamped message containing 3D coordinates wrt to the ebot_base frame
# Logic 		- The function uses cv2's ux_to_xyz function to get conversion from image 
# 				  coordinates to 3d world coordinates and then transform the world coorindates wrt to ebot_base
def uv_to_xyz(cx, cy):
    #Converting to XYZ coordinates
    (x, y, z) = cam_model.projectPixelTo3dRay((cx, cy))
    #Normalising
    x = x/z
    y = y/z
    z = z/z

    #Getting the depth at given coordinates
    depth = rospy.wait_for_message(depth_topic, Image)
    depth_img = img.frombytes("F", (depth.width, depth.height), depth.data)
    lookup = depth_img.load()
    d = lookup[cx, cy]

    #Modifying the coordinates
    x *= d
    y *= d
    z *= d

    #Making Point Stamp Message
    grasp_pose = geometry_msgs.msg.PointStamped()
    grasp_pose.header.frame_id = camera_frame
    grasp_pose.point.x =  x
    grasp_pose.point.y =  y
    grasp_pose.point.z =  z
	
	#Transforming
    target_frame = "ebot_base"
    source_frame = "camera_rgb_frame2"
    transform = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
   
    pose_transformed = tf2_geometry_msgs.do_transform_point(grasp_pose, transform)
    #Returning the transform coordinates
    return pose_transformed


# Function to give the 3D pose of the given object
# Arguments		- name of the required object
# Return Value 	- Pose Stamped message giving the 3D pose of the object, if the object was identified, else None
# Logic 		- The function uses YOLOv3 _Tiny object detector to localize the object in the image
# 				  The 2D image localization is then converted to 3D coordinates wrt to ebot_base using uv_to_xyz function 
def yolo(obj_name):
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
	xy_pose = None
	idxs = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.3)

	#Ensuring at least one detection exists
	if len(idxs) > 0:
		#Looping over the indexes
		for i in idxs.flatten():
			#Extracting the bounding box coordinates
			(x, y) = (boxes[i][0], boxes[i][1])
			(w, h) = (boxes[i][2], boxes[i][3])
			#Getting the labels
			label = LABELS[classIDs[i]]

			#If we get the required label
			if label == obj_name:

				#Getting center coordinates
				cx = x + (w/2)
				cy = y + (h/2)

				#Converting the center cooridnates to base link frame
				xy_pose = uv_to_xyz(cx, cy)

				#Publishing the pose
				xy_pose.header.frame_id = label
		
		#Looping over the indexes
		for i in idxs.flatten():
			#Extracting the bounding box coordinates
			(x, y) = (boxes[i][0], boxes[i][1])
			(w, h) = (boxes[i][2], boxes[i][3])
			
			#Getting the labels
			label = LABELS[classIDs[i]]
			
			#Print coloreding the labels and drawing bounding box on image
			print colored("{}  Identified".format(label), "blue")
			color = (0,0,255)
			cv2.rectangle(image, (x , y), (x + w, y + h ), color, 2)
			text = "{}".format(label)
			cv2.putText(image, text, (x, y - 20), cv2.FONT_HERSHEY_SIMPLEX,0.5, color, 2)

		#Showing the image
		cv2.imshow("output",image)
		cv2.waitKey(0)
		cv2.destroyAllWindows()
		return xy_pose


# Function to navigate to a given point
# Arguments 	- X, Y, Z and Roll, Pitch and Yaw of the goal location
# Return Value 	- None
# Logic 		- Passes the given coordinates to MoveBase as goal, and thus navigates to the given coordinates
def navigate(nav_x,nav_y,nav_r,nav_p,nav_yaw):
	#Transforming for euler angles to quaternion
	q = tf.transformations.quaternion_from_euler(nav_r,nav_p,nav_yaw)
	rospy.loginfo('Starting Navigation')
	
	# Defining MoveBase action server
	navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	navclient.wait_for_server()

	# Defining the goal location
	goal= MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()

	goal.target_pose.pose.position.x = nav_x
	goal.target_pose.pose.position.y = nav_y
	goal.target_pose.pose.position.z = 0
	goal.target_pose.pose.orientation.x = q[0]
	goal.target_pose.pose.orientation.y = q[1]
	goal.target_pose.pose.orientation.z = q[2]
	goal.target_pose.pose.orientation.w = q[3]

	# Sending goal to the action server
	navclient.send_goal(goal)
	navclient.wait_for_result()


# Function to pick object
# Arguments		- Object to pick
#				- Gripper closing to grasp the object
# Return Values - None
# Logic 		- The function takes in the object which has to pick up, and the amount by which gripper is supposed to close to grasp that object
#				  It then estimates the 3D pose of the object using YOLO function and grasps the object by moving the arm using Moveit
def pick_object(object,gripper_v,coke_closure=0):

	

	#Moving the arm to camera pose, to get the 3D pose of the object 
	if (object == 'Glue'):
		arm_group.set_named_target("FPGA_int_pose")
		arm_group.go(wait=True)
		arm_group.set_named_target("Meeting_room_camera_pose")
		arm_group.go(wait=True)	
	else:
		arm_group.set_named_target("FPGA_int_pose")
		arm_group.go(wait=True) 
		arm_group.set_named_target("camera_pose")
		arm_group.go(wait=True)

	rospy.sleep(2)

	#Getting the 3D pose of the object
	point_target = yolo(object)


	if (object == 'FPGA_Board'):
		arm_group.set_named_target("FPGA_int_pose")
		arm_group.go(wait=True)
	
	elif(object == 'Coke' or object == 'Battery'):
		arm_group.set_named_target("Coke_int_pose")
		arm_group.go(wait=True)
	else:
		pass


	# Opening gripper
	hand_group.set_named_target("gripper_open")
	# Sending the values to the robot for execution
	hand_group.go(wait=True)

	#Defining a Pose Stamped message as the goal for Arm to pick the object
	pose_target = geometry_msgs.msg.PoseStamped()
	pose_target.header.frame_id = "ebot_base"
	
	if (object == 'Glue'): 
		pose_target.pose.position.x = point_target.point.x - 0.01
		pose_target.pose.position.y = point_target.point.y - 0.20 
		pose_target.pose.position.z = point_target.point.z + 0.05


	elif (object == 'FPGA_Board'):

		if (point_target.point.y < -0.1): 
			pose_target.pose.position.x = point_target.point.x - 0.2
			pose_target.pose.position.y = point_target.point.y + 0.10
			pose_target.pose.position.z = point_target.point.z
		elif (point_target.point.y >= -0.1 and point_target.point.y <=0.1):
			pose_target.pose.position.x = point_target.point.x - 0.2
			pose_target.pose.position.y = point_target.point.y + 0.115
			pose_target.pose.position.z = point_target.point.z
		else: 
			pose_target.pose.position.x = point_target.point.x - 0.2
			pose_target.pose.position.y = point_target.point.y + 0.13
			pose_target.pose.position.z = point_target.point.z
	elif (object == 'Battery'):

		pose_target.pose.position.x = point_target.point.x - 0.2
		pose_target.pose.position.y = point_target.point.y 
		pose_target.pose.position.z = point_target.point.z

	else:

		if (point_target.point.y < -0.2):
			pose_target.pose.position.x = point_target.point.x - 0.2
			pose_target.pose.position.y = point_target.point.y - 0.0127
			pose_target.pose.position.z = point_target.point.z + 0.04
		elif (point_target.point.y >= -0.2 and point_target.point.y <= 0.2):
			pose_target.pose.position.x = point_target.point.x - 0.2
			pose_target.pose.position.y = point_target.point.y - 0.007
			pose_target.pose.position.z = point_target.point.z + 0.06
		else: 
			pose_target.pose.position.x = point_target.point.x - 0.2
			pose_target.pose.position.y = point_target.point.y + 0.004
			pose_target.pose.position.z = point_target.point.z + 0.04
			
		

	pose_target.pose.orientation.x = arm_group.get_current_pose().pose.orientation.x
	pose_target.pose.orientation.y = arm_group.get_current_pose().pose.orientation.y
	pose_target.pose.orientation.z = arm_group.get_current_pose().pose.orientation.z
	pose_target.pose.orientation.w = arm_group.get_current_pose().pose.orientation.w

	# Specifying the target pose to the planning group of the robot.
	arm_group.set_pose_target(pose_target)
	# Giving the robot a maximum planning time 20 sec to plan its path.
	arm_group.set_planning_time(20)
	# Planning a path to reach the location
	plan1 = arm_group.plan()
	# Executing the path
	arm_group.go(wait=True)

	# The robot has to moves by a specific amount to ensure that the object is in between the gripper fingers.
	# Empty list for waypoints to be followed.
	waypoints = []

	# Getting the current pose of the robot, essential as the robot would then iterate cartesian path from the that pose.
	wpose = arm_group.get_current_pose().pose

	# Move sideways (y) by 10cm
	if (object == 'Glue'):
		wpose.position.y +=  0.105  
		waypoints.append(copy.deepcopy(wpose))

	if (object == 'Battery'):
		wpose.position.x +=  0.08  
		waypoints.append(copy.deepcopy(wpose))

	elif (object == 'FPGA_Board'):
		wpose.position.y -=  0.05 
		wpose.position.x +=  0.100  
		waypoints.append(copy.deepcopy(wpose))


	else:
		wpose.position.x +=  coke_closure  
		waypoints.append(copy.deepcopy(wpose))

	# We want the Cartesian path to be interpolated at a resolution of 1 cm. Thus we specify eef_step as 0.01. 
	# We disable the jump threshold by setting it to 0.0.
	(plan, fraction) = arm_group.compute_cartesian_path(
									waypoints,   # waypoints to follow
									0.01,        # eef_step
									0.0) 

	# Sending the plan to the robot for execution.
	arm_group.execute(plan,wait=True)

	# Closing the gripper by specific amount 
	# Obtaining the current value of the gripper 
	group_variable_values = hand_group.get_current_joint_values()

	# Moving the gripper to a specific joint value
	group_variable_values[0] = gripper_v

	hand_group.set_joint_value_target(group_variable_values)

	# Sending the values to the robot for execution
	hand_group.go(wait=True)


	# The robot moves up by a specific amount before moving towards goal location to avoid collisions with other objects.
	waypoints = []
	wpose = arm_group.get_current_pose().pose

	# Move up (z) by 20 cm
	wpose.position.z +=  0.1   
	waypoints.append(copy.deepcopy(wpose))
	(plan, fraction) = arm_group.compute_cartesian_path(
									waypoints,   # waypoints to follow
									0.01,        # eef_step
									0.0) 
	arm_group.execute(plan,wait=True)

	arm_group.set_named_target("FPGA_int_pose")
	# Sending the arm to execute a path to that specific location
	arm_group.go(wait=True)
	#Moving the arm to init pose for smooth navigation
	arm_group.set_named_target("init_pose")
	arm_group.go(wait=True)

# Function to place/drop object in drop box
# Arguments		- Object to drop
# Return Value 	- None
# Logic			- The function places the given object in the drop box
def place_object(room):
	# Move the robot to drop location 
	# Obtaining the robot's current joint values.
    
	arm_group.set_named_target("FPGA_int_pose")
	arm_group.go(wait=True)

	arm_group.set_named_target(room+"_drop_pose")
	arm_group.go(wait=True)

	# Executing the path
	arm_group.go(wait=True)
	
	# Opening the gripper to drop the object.
	hand_group.set_named_target("gripper_open")
	hand_group.go(wait=True)

	#Going back to map pose to avoid collisions 
	arm_group.set_named_target("FPGA_int_pose")
	arm_group.go(wait=True)

	#Going to init pose to ensure smooth navigation
	arm_group.set_named_target("init_pose")
	# Sending the arm to execute a path to that specific location.
	arm_group.go(wait=True)


# Function to place/drop object in drop box
# Arguments 	- None
# Return Value 	- None
# Logic 		- In the pantry the robot looks first towards its left, to determine which table coke is spwaned on
#				  Based on the above gained knowledge it plans the path to respective table and picks the can using pick_object function
def pantry():
	#Going to pose to find which table coke can is spwaned on
	arm_group.set_named_target("pantry_pose_1")
	# Sending the joint set to the arm to execute a path to that specific location.
	arm_group.go(wait=True)

	#Checking coke can is on left table or right
	#If YOLO returns None, its on the right table, else on the left table
	if (yolo("Coke") is not None):
		#Going to init pose to ensure smooth navigation
		arm_group.set_named_target("init_pose")
		arm_group.go(wait=True)

		#Navigating to positon to pick coke can
		navigate(Dict['Pantry']['location'][2][0], Dict['Pantry']['location'][2][1], Dict['Pantry']['location'][2][2],Dict['Pantry']['location'][2][3], Dict['Pantry']['location'][2][4])

		#Picking the object
		pick_object('Coke',Object_dict['Coke'], 0.090)
		
	else:
		#Going to init pose to ensure smooth navigation
		arm_group.set_named_target("init_pose")
		arm_group.go(wait=True)

		#Navigating to positon to pick coke can
		navigate(Dict['Pantry']['location'][3][0], Dict['Pantry']['location'][3][1], Dict['Pantry']['location'][3][2],Dict['Pantry']['location'][3][3], Dict['Pantry']['location'][3][4])

		#Picking the object
		pick_object('Coke',Object_dict['Coke'], 0.077)


# Main thread
if __name__ == '__main__':

	#Subscring to image from gripper mounted real sense camera
	rospy.Subscriber(image_topic, Image, get_image) 
	
	global Dict, Object_dict

	#Dictionary having the values of various waypoints in different rooms
	Dict = {'Store_room':{'location':[[25.900, -3.0452, 0, 0, -0.66],[24.985, -4.123, 0, 0, -2.763030]]}, 
			'Meeting_room':{'location':[[8.671,1.0795,0,0,1.5707],[7.406747, 2.484625, 0, 0, 0],[8.671,2.484625,0,0,-1.5707]], 'pose':[-0.20355, 0.66785, 0.98789, 0.12520,-0.7143,-0.67655,0.12781]},  
			'Research_lab':{'location':[[11.608716, 5.760000,0,0,1.5707],[10.8279, 9.8527, 0, 0, -1.5707]], 'pose':[0.38526, 0.51656, 1.02126, 0.196038,-0.67939,-0.66485,0.240738]}, 
			'Conference_room':{'location':[[5.304,0.997,0,0,-1.5707],[5.4,-0.208,0,0,1.5707],[5.237,-0.294,0,0,1.5707]], 'pose':[-0.63793, -0.16059, 1.0544, 0.5584,-0.4071,-0.43247,0.57910]}, 
			'Pantry':{'location':[[13.0541,1,0,0,-1.5707],[13.0530, -0.2780, 0, 0, -1.5707],[14.423842, -0.813338, 0,0,0],[11.3853,-1.0456,0,0,-3.14],[13.0530, -0.2780, 0, 0, 1.5707]]},
			'Home':{'location':[[0,0,0,0,3.142]]}}
	
	#Dictionary having the values of gripper closing for various objects
	Object_dict = {'FPGA_Board': 0.250,'Glue':0.230,'Coke': 0.230, 'Battery': 0.31}

	#Pick up order: Battery, Coke can
	# Store_room, Research lab(drop Battery), Pantry (get Coke), Conference room (drop the Coke) 
	print colored("Started Run!", "blue")
	
	
	#Navigating to store room to pick Battery
	navigate(Dict['Store_room']['location'][0][0], Dict['Store_room']['location'][0][1], Dict['Store_room']['location'][0][2],Dict['Store_room']['location'][0][3], Dict['Store_room']['location'][0][4])
	
	print colored("Store Room Reached", "blue")
	
	#Picking up Battery 
	pick_object('Battery',Object_dict['Battery'])

	print colored("Battery Picked", "blue")

	navigate(Dict['Store_room']['location'][1][0], Dict['Store_room']['location'][1][1], Dict['Store_room']['location'][1][2],Dict['Store_room']['location'][1][3], Dict['Store_room']['location'][1][4])
	
	#Navigating to Research lab to place Battery
	navigate(Dict['Research_lab']['location'][0][0], Dict['Research_lab']['location'][0][1], Dict['Research_lab']['location'][0][2],Dict['Research_lab']['location'][0][3], Dict['Research_lab']['location'][0][4])
	navigate(Dict['Research_lab']['location'][1][0], Dict['Research_lab']['location'][1][1], Dict['Research_lab']['location'][1][2],Dict['Research_lab']['location'][1][3], Dict['Research_lab']['location'][1][4])
	print colored("Research Lab Reached", "blue")


	#Placing the Battery
	place_object('Research_lab')
	
	print colored("Battery Dropped in Research Lab (DropBox3)", "blue")
	
	#Navigating to Pantry to Pick Coke can
	navigate(Dict['Pantry']['location'][0][0], Dict['Pantry']['location'][0][1], Dict['Pantry']['location'][0][2],Dict['Pantry']['location'][0][3], Dict['Pantry']['location'][0][4])
	navigate(Dict['Pantry']['location'][1][0], Dict['Pantry']['location'][1][1], Dict['Pantry']['location'][1][2],Dict['Pantry']['location'][1][3], Dict['Pantry']['location'][1][4])         
	
	print colored("Pantry Reached", "blue")
	
	#Checking the positon on Coke can in Pantry, and pick it up
	pantry()

	print colored("Coke Picked", "blue")
	
	#Navigating out of Pantry and to Conference Room to place Coke can 
	navigate(Dict['Pantry']['location'][4][0], Dict['Pantry']['location'][4][1], Dict['Pantry']['location'][4][2],Dict['Pantry']['location'][4][3], Dict['Pantry']['location'][4][4])
	
	
	navigate(Dict['Conference_room']['location'][0][0], Dict['Conference_room']['location'][0][1], Dict['Conference_room']['location'][0][2],Dict['Conference_room']['location'][0][3], Dict['Conference_room']['location'][0][4])
	navigate(Dict['Conference_room']['location'][1][0], Dict['Conference_room']['location'][1][1], Dict['Conference_room']['location'][1][2],Dict['Conference_room']['location'][1][3], Dict['Conference_room']['location'][1][4])
	
	#Placing Coke Can
	place_object('Conference_room')
	
	print colored("Coke Dropped in Conference Room (DropBox-1)", "blue")

	navigate(Dict['Home']['location'][0][0], Dict['Home']['location'][0][1], Dict['Home']['location'][0][2],Dict['Home']['location'][0][3], Dict['Home']['location'][0][4])
	
	print colored("Mission Accomplished!", "blue")
	# Waiting indefinitely
	rospy.spin()	
