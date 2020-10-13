#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg     import Pose
import time
import math
from std_srvs.srv import Empty



class Movement():

	def __init__(self):
		self.dist_moved=0
		self.x=0
		self.y=0
		self.init_yaw=0
		self.yaw=0
        	self.flag=0
        	self.t0 = 0
        	self.t1 = 0
		rospy.init_node('node_turtle_revolve',anonymous=True)
		self.vel_pub=rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
		self.pose_subs=rospy.Subscriber('/turtle1/pose', Pose, self.PoseCallback)
		self.vel_msg = Twist()

	def PoseCallback(self,pose_message):
		self.yaw = pose_message.theta

	def stop_turtle(self):
		self.vel_msg.angular.z=0
		self.vel_msg.linear.x=0
		rospy.loginfo("Goal Reached")
		self.vel_pub.publish(self.vel_msg)
	
	def dist_moved_circle(self):
		self.dist_moved = abs(self.speed * (self.t0 - self.t1))
		return self.dist_moved
	
	def move(self,speed,radius,is_forward):
		self.t0 = rospy.Time.now().to_sec()
		self.speed = speed
		self.radius = radius
		dist_moved = 0
		if(is_forward):
			self.vel_msg.linear.x = abs(self.speed)
			self.vel_msg.angular.z = self.speed/self.radius
		else:
			self.vel_msg.linear.x= -abs(self.speed)
			self.vel_msg.angular.z= -(self.speed/self.radius)

		
		while not rospy.is_shutdown():
			self.t1 = rospy.Time.now().to_sec()
			rospy.loginfo("Moving in a circle")
			self.vel_pub.publish(self.vel_msg)
			dist_moved =self.dist_moved_circle()
			print(dist_moved)
			#print("YAW",self.yaw)	
			if (int(self.yaw)==-3):
				self.flag=1
				#print("First condition")
			
			if ((round(self.yaw,2)- self.init_yaw)>=-0.05 and self.flag==1):
				self.flag=2	
				#print(round(self.yaw,2)- self.init_yaw)
				#print("Second condition")
			#print("FLAG",self.flag)
			
			if (self.flag==2):
				self.stop_turtle()
				break



if __name__=='__main__':
	robot= Movement()
	try:
		time.sleep(2)
		print("Move")
		robot.move(10.0,2.0,True)
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("Node Terminated")
