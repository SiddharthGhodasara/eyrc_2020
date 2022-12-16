#! /usr/bin/env python

import sys
import copy
import rospy
import math
import tf
import time
import moveit_commander 
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, JointConstraint, Constraints
import actionlib


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_python_node',anonymous=True)

robot = moveit_commander.RobotCommander()  
scene = moveit_commander.PlanningSceneInterface()

arm_group = moveit_commander.MoveGroupCommander("arm")
hand_group= moveit_commander.MoveGroupCommander("gripper")


display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path' , moveit_msgs.msg.DisplayTrajectory,queue_size=20)


pose_target = geometry_msgs.msg.PoseStamped()
x , y, z, w = quaternion_from_euler(-2.62228810952, 0.119908924743, -3.13559289599)
pose_target.header.frame_id = "ebot_base"
pose_target.pose.position.x = 0.270670484391
pose_target.pose.position.y = 0.434277321698
pose_target.pose.position.z = 0.93

pose_target.pose.orientation.x = x
pose_target.pose.orientation.y = y
pose_target.pose.orientation.z = z
pose_target.pose.orientation.w = w

#arm_group.set_goal_tolerance(0.007)
arm_group.set_pose_target(pose_target)
#group.set_path_constraints(upright_constraints)
arm_group.set_planning_time(20)
plan = arm_group.plan()
plan1 =arm_group.go(wait=True)


moveit_commander.roscpp_shutdown()
