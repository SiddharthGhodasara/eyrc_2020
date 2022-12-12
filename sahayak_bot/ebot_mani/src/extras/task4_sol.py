#!/usr/bin/env python


'''
TEAM ID: SB#337

TEAM MEMBERS: SIDDHARTH GHODASARA (TEAM LEADER)
              GAURAV SETHIA
			  HARISH GURAGOL
			  VINEET RANJAN
	      
CODE DESCRIPTION: -> This code is responsible for completing task 3, ie. moving a UR5 robotic arm using Moveit to pick and place 3 objects
                     while avoiding obstacles.

                  -> The following steps are taken to ensure the robot performs this task:
                     STEP 1: Move the robot to hover above the object.
                     STEP 2: The robot then moves down to keep the object between its fingers.
                     STEP 3: It then closes the gripper thus grasping the object.
                     STEP 4: Moves up whilst holding the object.
                     STEP 5: Moves to the drop location.
                     STEP 6: Drops the object.  
                     
                     The above steps take place for each object.

PUBLISHER: "/move_group/display_planned_path"     MESSAGE TYPE: moveit_msgs.DisplayTrajectory
 
GLOBAL VARIABLES : robot      : Instantiating a RobotCommander object. This provides information on robot's current joint states.
                   scene      : Instantiating a PlanningScene object(provides info on obtaining and updating robot's understanding of the world)
                   arm_group  : Instantiating a MoveGroupCommander object. This object is an interface to a planning group for the robot
                   hand_group : Instantiating a MoveGroupCommander object. This object is an interface to a planning group for the robot's gripper


MEMBER FUNCTIONS : move_arm: Responsible for moving the arm to a given loaction
'''

# Importing libraries
import sys
import copy
import moveit_commander 
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal

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
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel


#Defining the bridge
bridge = CvBridge()

#Defining Class
class pick_and_place:

    #Init Function
    def __init__(self):

        # Initializing moveit_commander 
        moveit_commander.roscpp_initialize(sys.argv)

        # Instantiating a RobotCommander object. This provides information on robot's current joint states.
        robot = moveit_commander.RobotCommander() 

        # Instantiating a PlanningScene object(provides info on obtaining and updating robot's understanding of the world)
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiating a MoveGroupCommander object. This object is an interface to a planning group.
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.hand_group= moveit_commander.MoveGroupCommander("gripper")

        # Create a DisplayTrajectory ROS publisher (used to display trajectories in Rviz)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path' , moveit_msgs.msg.DisplayTrajectory,queue_size=20)

        # Moving the robot to init_pose
        self.arm_group.set_named_target('init_pose')
        plan = self.arm_group.go(wait=True)

        # Defining Camera Topics
        self.image_topic = '/camera/color/image_raw2'
        self.camera_topic = '/camera/color/camera_info2'
        self.depth_topic = '/camera/depth/image_raw2'
        self.camera_frame = 'camera_link2'

        #Getting camera info
        self.camera_info = rospy.wait_for_message(self.camera_topic, CameraInfo)
        #Setting up Camera Model
        self.cam_model = PinholeCameraModel()
        self.cam_model.fromCameraInfo(self.camera_info)
        #print(self.camera_info)
        #Directoanry to store pose
        self.pose_dir = {}

        #Tranform Listener
        self.tf_listener = None
        #Transformations
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length

        #Loading the labels
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('ebot_mani')
        labelsPath = os.path.sep.join([package_path, 'yolo', 'obj.names'])
        self.LABELS = open(labelsPath).read().strip().split("\n")

        #Derive the paths to the YOLO weights and model configuration
        weightsPath = os.path.sep.join([package_path, 'yolo', "yolov3-tiny-obj_1000.weights"])
        configPath = os.path.sep.join([package_path, 'yolo', "yolov3-tiny-obj.cfg"])

        # load our YOLO object detector trained on custom data
        print("[INFO] loading YOLO from disk...")
        self.net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)


        #Initialising Transformations
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        #Initialising the publisher
        self.pub = rospy.Publisher('/detection_info', geometry_msgs.msg.PoseStamped, queue_size = 10)


        

    #Function for converting coordinates to base link frame
    def uv_to_xyz(self, cx, cy):
        #Converting to XYZ coordinates
        (x, y, z) = self.cam_model.projectPixelTo3dRay((cx, cy))
        #Normalising
        x = x/z
        y = y/z
        z = z/z

        #Getting the depth at given coordinates
        depth = rospy.wait_for_message(self.depth_topic, Image)
        #print(type(depth))
        depth_img = img.frombytes("F", (depth.width, depth.height), depth.data)
        lookup = depth_img.load()
        d = lookup[cx, cy]

        #Modifying the coordinates
        x *= d
        y *= d
        z *= d

        #Making Point Stamp Message
        grasp_pose = geometry_msgs.msg.PoseStamped()
        grasp_pose.header.frame_id = self.camera_frame
        grasp_pose.pose.position.x =  x
        grasp_pose.pose.position.y =  y
        grasp_pose.pose.position.z =  z
        grasp_pose.pose.orientation.x = 0
        grasp_pose.pose.orientation.y = 0
        grasp_pose.pose.orientation.z = 0
        grasp_pose.pose.orientation.w = 1

        #Transforming
        target_frame = "base_link"
        source_frame = self.camera_frame
        transform = self.tf_buffer.lookup_transform(target_frame,
                                            source_frame, #source frame
                                            rospy.Time(0), #get the tf at first available time
                                            rospy.Duration(1.0)) #wait for 1 second

        #Applying the transform
        pose_transformed = tf2_geometry_msgs.do_transform_pose(grasp_pose, transform)
        #Returning the transform coordinates
        return grasp_pose

    
    #Prediction Function
    def predict_pose(self):

        #Getting the image
        print("Waiting for images")
        img = rospy.wait_for_message(self.camera_topic, Image)
        print("Obtained images")
        image = bridge.imgmsg_to_cv2(img, 'bgr8')

        #Getting the image dimensions
        (H, W) = image.shape[:2]

        # determine only the *output* layer names that we need from YOLO
        ln = self.net.getLayerNames()
        ln = [ln[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        #Creating Blob from images
        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        #Feeding the blob as input
        self.net.setInput(blob)
        #Forward pass
        layerOutputs = self.net.forward(ln)

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
                label = self.LABELS[classIDs[i]]
                print(label)

                #Getting center coordinates
                cx = x + (w/2)
                cy = y + (h/2)
                #Converting the center cooridnates to base link frame
                xy_pose = self.uv_to_xyz(cx, cy)

                xy_pose.header.frame_id = label

                self.pose_dir[label] = [xy_pose.pose.position.x, xy_pose.pose.position.y, xy_pose.pose.position.z, xy_pose.pose.orientation.x, xy_pose.pose.orientation.y, xy_pose.pose.orientation.z, xy_pose.pose.orientation.w]

                #Publishing
                self.pub.publish(xy_pose)
                print("Published")


    # Function to move the arm 
    # Arguments:- 6D Pose of the location (position and orientation(in qauternion))
    #           - Gripper's closing value
    #           - Joint value of the robot (waist joint of the robot) which is unique and thus is given individually (we have used joint angles for diversify
    #             from the conventional 6D pose to specifiy goal location. The same output was found when feeding moveit with 6D poses.)
    def move_arm(self, pos_x, pos_y, pos_z, or_x, or_y, or_z, or_w, gripper_v):

        # STEP 1 
        # Move the robot to hover above the object.
        # Giving the robot a 6D pose on the object location.
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = pos_x
        pose_target.position.y = pos_y
        pose_target.position.z = pos_z

        pose_target.orientation.x = or_x
        pose_target.orientation.y = or_y
        pose_target.orientation.z = or_z
        pose_target.orientation.w = or_w

        # Specifying the target pose to the planning group of the robot.
        self.arm_group.set_pose_target(pose_target)

        # Giving the robot a maximum planning time 20 sec to plan its path.
        self.arm_group.set_planning_time(20)

        # Planning a path to reach the location
        plan1 = self.arm_group.plan()

        # Executing the path
        self.arm_group.go(wait=True)


        # STEP 2
        # The robot has to go down by a specific amount to ensure that the object is in between the gripper fingers.
        # Empty list for waypoints to be followed.
        waypoints = []

        # Getting the current pose of the robot, essential as the robot would then iterate cartesian path from the that pose.
        wpose = self.arm_group.get_current_pose().pose

        # Move sideways (y) by 10cm
        wpose.position.y +=  0.1  
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm. Thus we specify eef_step as 0.01. 
        # We disable the jump threshold by setting it to 0.0.
        (plan, fraction) = self.arm_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0) 

        # Sending the plan to the robot for execution.
        self.arm_group.execute(plan,wait=True)


        # STEP 3
        # Closing the gripper by specific amount 
        # Obtaining the current value of the gripper 
        group_variable_values = self.hand_group.get_current_joint_values()

        # Moving the gripper to a specific joint value
        group_variable_values[0] = gripper_v

        self.hand_group.set_joint_value_target(group_variable_values)

        # Sending the values to the robot for execution
        self.hand_group.go(wait=True)


        # STEP 4
        # The robot then proceeds to move up by a specific amount before moving towards goal location.
        # This cartesian path code is as explained above.
        waypoints = []

        wpose = self.arm_group.get_current_pose().pose

        # Move up (z) by 20 cm
        wpose.position.z +=  0.2  

        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.arm_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0) 
        self.arm_group.execute(plan,wait=True)


        # STEP 5
        # Move the robot to drop location 
        # Obtaining the robot's current joint values.
        self.arm_group.set_named_target("drop_pose")

        # Sending the joint set to the arm to execute a path to that specific location.
        self.arm_group.go(wait=True)


        # STEP 6
        # Opening the gripper to drop the object.
        self.hand_group.set_named_target("gripper_open")
        self.hand_group.go(wait=True)


    

#Main Thread
if __name__ == "__main__":
    # Initializing rospy node
    rospy.init_node('move_python_node',anonymous=True)

    #Creating Object
    task_obj = pick_and_place()

    #Predicting the pose of all the objects 
    task_obj.predict_pose()

    print(task_obj.pose_dir)

