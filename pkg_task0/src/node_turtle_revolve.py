#!/usr/bin/env python
'''
TEAM ID: SB#337

TEAM MEMBERS: SIDDHARTH GHODASARA (TEAM LEADER)
              GAURAV SETHIA
	      HARISH GURAGOL
	      VINEET RANJAN
CODE DESCRIPTION: The following code is written to move the turtle move in a circle and stop after one complete revolution

PUBLISHER: /turtle1/cmd_vel

SUBSCRIBER: /turtle1/pose

CLASS VARIABLES: t0 - Stores the start time 
                 t1 - Stores the current time
		 yaw - Stores the current yaw/orientation of the turtle
		 init_yaw - Stores the initial yaw/orientation of the turtle
		 speed - Stores the speed of the turtle
		 flag - This variable is used as a counter, that is set to 1 as the turtle completes half a circle and is set to 2 after a complete revolution 
		 dist_moved - Stores the distance moved by the turtle
		 
CLASS MEMBER FUNCTIONS : PoseCallback - Callback function for pose subscriber
                         stop_turtle - Stops the turtle by setting the velocity to 0
                         dist_moved_circle - Computes the distance travelled by the robot using speed and time (Distance = Speed * Time)
                         move - Move in a circle of given radius until it reaches the same starting orientation		 
'''

# Importing Libraries
import time
import math
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

# Class to control the turtle's movements and keep a track of distance moved
class Movement():
	
	# Initialization function
	# Logic - Initializes all the required variables
	
	def __init__(self):
		# Initializing the required variables
		self.t0 = 0
		self.t1 = 0
		self.yaw = 0
		self.flag = 0
		self.speed = 0
		self.init_yaw = 0
		self.dist_moved = 0
		# Instantiating a geometry_message/Twist object
		self.vel_msg = Twist()
		# Initializing the node
		rospy.init_node('node_turtle_revolve',anonymous=True)
		#Defining the velocity publisher
		self.vel_pub=rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
		# Defining the pose subscriber
		self.pose_subs=rospy.Subscriber('/turtle1/pose', Pose, self.PoseCallback)

	# Function Name - PoseCallback
	# Arguments - pose_message (message sent by the publisher)
	# Callback Function for Pose
	# Logic - Subscribes to the data that is being published by the turtle
	
	def PoseCallback(self, pose_message):
		# Storing the turtle rotation value
		self.yaw = pose_message.theta

	# Function Name - stop_turtle
	# Arguments - None
	# Function to stop the turtle
	# Logic - Sets the Velocity of the turtle to 0
	
	def stop_turtle(self):
		# Making linear velocity 0
		self.vel_msg.linear.x=0
		# Making angular velocity 0
		self.vel_msg.angular.z=0
		# Publishing the velocity message
		self.vel_pub.publish(self.vel_msg)
		rospy.loginfo("Goal Reached")

	# Function Name - dist_moved_circle
	# Arguments - None
	# Function to find the distance moved by the turtle
	# Logic - Computes the distance moves by the robot based on speed and time
	
	def dist_moved_circle(self):
		self.dist_moved = abs(self.speed * (self.t0 - self.t1))
		return self.dist_moved

	# Function Name - move
	# Arguments - Speed (Speed of the turtle), Radius (Radius of circle), is_forward (Setting direction)
	# Function to move the turtle in a circle
	# Logic - Move in a circle of given radius until it reaches the same orientation, as in the beginning
	
	def move(self,speed,radius,is_forward):
		# Storing the start time
		self.t0 = rospy.Time.now().to_sec()
		# Storing speed and radius to class variables
		self.speed = speed
		self.radius = radius
		dist_moved = 0

		# Assigning the direction of movement (clockwise or anticlockwise)
		# If is_forward is True, we move in anticlockwise direction or forward, else in clockwise direction or backwards
		if(is_forward):
			self.vel_msg.linear.x = abs(self.speed)
			self.vel_msg.angular.z = self.speed/self.radius
		else:
			self.vel_msg.linear.x= -abs(self.speed)
			self.vel_msg.angular.z= -(self.speed/self.radius)

		# Running until there is an keyboard interrupt
		while not rospy.is_shutdown():
			# Storing the current time
			self.t1 = rospy.Time.now().to_sec()
			# Logging
			rospy.loginfo("Moving in a circle")
			# Publishing the velocity, ie. making the turtle move
			self.vel_pub.publish(self.vel_msg)
			# Computing the distance moved
			dist_moved =self.dist_moved_circle()
			# Logging
			print(dist_moved)
			# Checking if the turtle has completed a semi-circle
			# If yes, then set the flag to 1
			if (int(self.yaw)==-3):
				self.flag=1

			# Checking if turtle has completed the complete circle and flag is 1
			# Set the flag to 2
			if ((round(self.yaw,2) - self.init_yaw)>=-0.05 and self.flag==1):
				self.flag=2

			# Stopping the turtle if flag is 2, which indicates it has reached the end point
			if (self.flag==2):
				self.stop_turtle()
				# Breaking out of while loop
				break

# Main Thread
if __name__=='__main__':
	# Creating a class object
	robot= Movement()
	try:
		# Delay
		time.sleep(2)
		# Moving the robot
		robot.move(10.0,2.0,True)
		# Adding rospy.spin to ensure node is stopped from exiting
		rospy.spin()
	# Handling Keyboard Interrupts
	except rospy.ROSInterruptException:
		rospy.loginfo("Node Terminated")
