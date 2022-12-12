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
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel

import sys
import copy
import rospy
import tf
import time
import moveit_commander 
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal

#Getting the parameters from launch file
image_topic = '/camera/color/image_raw2'
camera_topic = '/camera/color/camera_info2'
depth_topic = '/camera/depth/image_raw2'
camera_frame = 'camera_link2'

#Defining the bridge
bridge = CvBridge()

#Declaring the publisher
pub = None
#Defining camera info global varibale
camera_info = None
cam_model = None
#Storing the depth
depth = None

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

    #Making Point Stamp Message
    grasp_pose = geometry_msgs.msg.PoseStamped()
    grasp_pose.header.frame_id = camera_frame
    grasp_pose.pose.position.x =  x
    grasp_pose.pose.position.y =  y
    grasp_pose.pose.position.z =  z

    #Transforming
    target_frame = "ebot_base"
    source_frame = camera_frame
    transform = tf_buffer.lookup_transform(target_frame,
                                           source_frame, #source frame
                                           rospy.Time(0), #get the tf at first available time
                                           rospy.Duration(1.0)) #wait for 1 second

    #Applying the transform
    pose_transformed = tf2_geometry_msgs.do_transform_pose(grasp_pose, transform)
    #Returning the transform coordinates
    print(pose_transformed)
    return pose_transformed


#Prediction Function
def prediction(image):
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
            #Converting the center cooridnates to base link frame
            xy_pose = uv_to_xyz(cx, cy)

            xy_pose.header.frame_id = label

            #Checking if subscribers are present
            while pub.get_num_connections() < 1:
                None
            #Publishing
            pub.publish(xy_pose)
            print("Published")


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

    #Initialising the node
    rospy.init_node("object_detection")

    #Getting camera info
    camera_info = rospy.wait_for_message(camera_topic, CameraInfo)
    #Setting up Camera Model
    cam_model = PinholeCameraModel()
    cam_model.fromCameraInfo(camera_info)

    #Initialising Transformations
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    #Initialising the publisher
    pub = rospy.Publisher('/detection_info', geometry_msgs.msg.PoseStamped, queue_size = 10)
    #Subscribinig to image topic
    sub = rospy.Subscriber(image_topic, Image, callback)

    #Stopping the node from termination
    rospy.spin()

#Calling the main thread
if __name__ == "__main__":
    main()
