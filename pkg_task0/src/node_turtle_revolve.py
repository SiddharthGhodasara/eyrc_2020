#!/usr/bin/env python

#Improting Libraries
import time
import math
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

#Class to control the robots movement and keep a track of distance moved
class Movement():
	#Initialization function
	#Logic - Initialises all the required varibales
	def __init__(self):
		#Initializing the reqired varibales
		self.t0 = 0
		self.t1 = 0
		self.yaw = 0
		self.flag = 0
		self.speed = 0
		self.init_yaw = 0
		self.dist_moved = 0
		self.vel_msg = Twist()
		#Initializing the node
		rospy.init_node('node_turtle_revolve',anonymous=True)
		#Defining the velocity publisher
		self.vel_pub=rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
		#Defining the pose subcriber
		self.pose_subs=rospy.Subscriber('/turtle1/pose', Pose, self.PoseCallback)

	#Function Name - PoseCallback
	#Arguments - pose_message(message sent by the publisher)
	#Callback Function for Pose
	#Logic - Subscribes to the data that is being published by the robot
	def PoseCallback(self, pose_message):
		#Storing the robots rotation value
		self.yaw = pose_message.theta

	#Function Name - stop_turtle
	#Arguments - None
	#Function to stop the turtle
	#Logic - Sets the Velocity of the robot to 0
	def stop_turtle(self):
		#Making linear velocity as 0
		self.vel_msg.linear.x=0
		#Making angular velocity as 0
		self.vel_msg.angular.z=0
		#Publishing the velocity
		self.vel_pub.publish(self.vel_msg)
		rospy.loginfo("Goal Reached")

	#Function name - dist_moved_circle
	#Arguments - None
	#Function to find the distance moved by the robot
	#Logic - Computes the distance moves by the robot based on speed and time
	def dist_moved_circle(self):
		self.dist_moved = abs(self.speed * (self.t0 - self.t1))
		return self.dist_moved

	#Function Name - move
	#Arguments - Speed (Speed of the robot), Radius (Radius of circle), is_forward (Setting direction)
	#Function to move the robot in a circle
	#Logic - Move in a circle of given radius until it reaches the same orientation, as in the beginning
	def move(self,speed,radius,is_forward):
		#Storing the start time
		self.t0 = rospy.Time.now().to_sec()
		#Storing speed and radius to class varibales
		self.speed = speed
		self.radius = radius
		dist_moved = 0

		#Assigning the direction of movement (clockwise or anticlockwise)
		# if is_forward is True, we move in anticlockwise direction or forward
		if(is_forward):
			self.vel_msg.linear.x = abs(self.speed)
			self.vel_msg.angular.z = self.speed/self.radius
		else:
			self.vel_msg.linear.x= -abs(self.speed)
			self.vel_msg.angular.z= -(self.speed/self.radius)

		#Running until there is an keyboard interrupt
		while not rospy.is_shutdown():
			#Storing the current time
			self.t1 = rospy.Time.now().to_sec()
			#Logging
			rospy.loginfo("Moving in a circle")
			#Publishing the velocity, ie, making the robot move
			self.vel_pub.publish(self.vel_msg)
			#Computing the distance moved
			dist_moved =self.dist_moved_circle()
			#Logging
			print(dist_moved)
			#Checking if the robot have completed semi cirlce
			#If yes, set the flag to 1
			if (int(self.yaw)==-3):
				self.flag=1

			#Checking if Robot has completed the complete circle and flag is 1
			#Set the flag to 2
			if ((round(self.yaw,2) - self.init_yaw)>=-0.05 and self.flag==1):
				self.flag=2

			#Stopping the robot if flag is 2, which indicates it has reached to end point
			if (self.flag==2):
				self.stop_turtle()
				#Breaking out of while loop
				break

#Main Thread
if __name__=='__main__':
	#Creating a class object
	robot= Movement()
	try:
		#Delay
		time.sleep(2)
		#Moving the robot
		robot.move(10.0,2.0,True)
		#Adding rospy.spin to ensure node is stopped from exiting
		rospy.spin()
	#Handling Keyboard Interrupts
	except rospy.ROSInterruptException:
		rospy.loginfo("Node Terminated")
