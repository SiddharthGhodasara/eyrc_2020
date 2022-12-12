#!/usr/bin/env python

'''
TEAM ID: SB#337

TEAM MEMBERS: SIDDHARTH GHODASARA (TEAM LEADER)
              GAURAV SETHIA
			  HARISH GURAGOL
			  VINEET RANJAN

CODE DESCRIPTION: -> This code is responsible for completing task 2, ie. following a given set of waypoints while avoiding obstacles.
                  -> The motivation behind the structure of this code was, again state machines ;) (simply because they are very effective and easy to implement).
                     This time we have moved a step forward and implemented smach_ros (a inbuilt library for state machines in ROS)
                     State machine description: State 1: Waypoint 1 (-9.1, -1.2)
                                                State 2: Waypoint 2 (10.7, 10.5)
                                                State 3: Waypoint 3 (12.6, -1.6) Originally: (12.6, -1.9)*
                                                State 4: Waypoint 4 (18.2, -1.4)
												State 5: Waypoint 5 (-2, 4)

                  * - The waypoint y = -1.9 was not reachable by the robot, hence we have taken y = -1.6 as the y co-ordinate with no change in x co-ordinate,
				      which satisfies the tolerance of +/- 0.5 in x and y direction. 

                  -> The logic of this code is fairly simple, keep following a set of waypoints as long as state returns 'Goal Reached'. If it returns 
				     anything else, abort the entire task and come out of the state container. If all tasks are done then return 'Task Finished'.

'''

# Importing standard libraries
import rospy
import time
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
		rospy.loginfo("Going to goal")


	def feedback_cb(self,feedback):
		rospy.loginfo("Current location " +str(feedback))

	
	def done_cb(self,status, result):
		self.stat=status
		return self.stat

	
	def execute(self, userdata):
		rospy.loginfo('Starting Navigation')
		
		navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		navclient.wait_for_server()

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

		navclient.send_goal(goal, self.done_cb , self.active_cb)
		finished = navclient.wait_for_result()
		
		if not finished:
			rospy.loginfo("Action server not available")
		
		elif self.stat == 3:
			return 'Goal_reached'
		
		elif self.stat == 2 or self.stat == 8:
			return 'Goal_cancelled'
		
		elif self.stat == 4:
			return 'Goal_aborted'


if __name__ == '__main__':
	# Initializing ROS node 
	rospy.init_node('ebot_state_machine',anonymous=True)


	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['TASK FINISHED','TASK ABORTED'])

	# Open the container
	with sm:
	# Add states to the container
		smach.StateMachine.add('Waypoint 1', Navigate(-9.1, -1.2), transitions={'Goal_reached':'Waypoint 2', 'Goal_cancelled':'TASK ABORTED', 'Goal_aborted':'TASK ABORTED'}) 
	
		smach.StateMachine.add('Waypoint 2', Navigate(10.7, 10.5), transitions={'Goal_reached':'Waypoint 3', 'Goal_cancelled':'TASK ABORTED', 'Goal_aborted':'TASK ABORTED'})

		smach.StateMachine.add('Waypoint 3', Navigate(12.6, -1.6), transitions={'Goal_reached':'Waypoint 4', 'Goal_cancelled':'TASK ABORTED', 'Goal_aborted':'TASK ABORTED'}) 
	
		smach.StateMachine.add('Waypoint 4', Navigate(18.2, -1.4), transitions={'Goal_reached':'Waypoint 5', 'Goal_cancelled':'TASK ABORTED', 'Goal_aborted':'TASK ABORTED'})

		smach.StateMachine.add('Waypoint 5', Navigate(-2.0, 4.0), transitions={'Goal_reached':'TASK FINISHED', 'Goal_cancelled':'TASK ABORTED', 'Goal_aborted':'TASK ABORTED'}) 
		
	# Execute SMACH plan
	outcome = sm.execute()

	# Waiting indefinitely
	rospy.spin()


		
