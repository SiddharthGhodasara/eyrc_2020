#!/usr/bin/env python
import rospy
import time
import geometry_msgs.msg
import actionlib
import smach
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# ----- NAVIGATION STACK ------#

class Navigate(smach.State):
	def __init__(self,nav_x,nav_y):
		smach.State.__init__(self, outcomes=['Goal_reached', 'Goal_cancelled', 'Goal_aborted'])
		self.nav_x=nav_x
		self.nav_y=nav_y	
	
	def active_cb(self):
		rospy.loginfo("Goal pose being processed")

	def feedback_cb(self,feedback):
		rospy.loginfo("Current location " +str(feedback))
	
	def done_cb(self,status, result):
		self.stat=status
		return self.stat

	
	def execute(self, userdata):
		rospy.loginfo('Starting Navigation')
		
		self.navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		self.navclient.wait_for_server()

		goal= MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()

		goal.target_pose.pose.position.x = self.nav_x
		goal.target_pose.pose.position.y = self.nav_y
		goal.target_pose.pose.position.z = 0
		#goal.target_pose.pose.orientation.x = 0
		#goal.target_pose.pose.orientation.y = 0
		#goal.target_pose.pose.orientation.z = 0.707 
		#goal.target_pose.pose.orientation.w = 0.707 
		goal.target_pose.pose.orientation.w = 1.0

		self.navclient.send_goal(goal, self.done_cb , self.active_cb)
		self.finished = self.navclient.wait_for_result()
		#print(self.stat)
		
		if not self.finished:
			rospy.loginfo("Action server not available")
		
		elif self.stat == 3:
			return 'Goal_reached'
		
		elif self.stat == 2 or self.stat == 8:
			return 'Goal_cancelled'
		
		elif self.stat == 4:
			return 'Goal_aborted'


if __name__ == '__main__':
	rospy.init_node('ebot_state_machine',anonymous=True)

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['TASK FINISHED','TASK ABORTED'])

	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()
	# Open the container
	with sm:
	# Add states to the container
	
		smach.StateMachine.add('Waypoint 2', Navigate(10.7, 10.5), transitions={'Goal_reached':'Waypoint 3', 'Goal_cancelled':'TASK ABORTED', 'Goal_aborted':'TASK ABORTED'})

		smach.StateMachine.add('Waypoint 3', Navigate(12.6, -1.6), transitions={'Goal_reached':'Waypoint 2', 'Goal_cancelled':'TASK ABORTED', 'Goal_aborted':'TASK ABORTED'}) 
	
		
		
	# Execute SMACH plan
	outcome = sm.execute()

	rospy.spin()
	sis.stop()

		
