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
from tf.transformations import quaternion_from_euler
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, JointConstraint, Constraints


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_python_node',anonymous=True)

robot = moveit_commander.RobotCommander()  
scene = moveit_commander.PlanningSceneInterface()

arm_group = moveit_commander.MoveGroupCommander("arm")
hand_group= moveit_commander.MoveGroupCommander("gripper")


display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path' , moveit_msgs.msg.DisplayTrajectory,queue_size=20)


# OBJECT 1 LOCATION
pose_target = geometry_msgs.msg.Pose()
pose_target.position.x = 0.57234
pose_target.position.y = -0.01541
pose_target.position.z = 0.791986

pose_target.orientation.x = -0.428910594342#q[0]
pose_target.orientation.y = -0.903346049818#q[1]
pose_target.orientation.z = 0.000156570721622#q[2]
pose_target.orientation.w = 0.00126167521046#q[3]

# arm_group.set_goal_tolerance(0.007)
arm_group.set_pose_target(pose_target)
#group.set_path_constraints(upright_constraints)
arm_group.set_planning_time(20)
plan1 = arm_group.plan()
arm_group.go(wait=True)

waypoints = []

wpose = arm_group.get_current_pose().pose
wpose.position.z -=  0.05  # First move up (z)

waypoints.append(copy.deepcopy(wpose))

# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0,
# ignoring the check for infeasible jumps in joint space, which is sufficient
# for this tutorial.
(plan, fraction) = arm_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0) 
arm_group.execute(plan,wait=True)

group_variable_values = hand_group.get_current_joint_values()
group_variable_values[0] = 0.2
hand_group.set_joint_value_target(group_variable_values)
hand_group.go(wait=True)


waypoints = []

wpose = arm_group.get_current_pose().pose
wpose.position.z +=  0.2  # First move up (z)

waypoints.append(copy.deepcopy(wpose))

# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0,
# ignoring the check for infeasible jumps in joint space, which is sufficient
# for this tutorial.
(plan, fraction) = arm_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0) 
arm_group.execute(plan,wait=True)


joint_goal = arm_group.get_current_joint_values()
joint_goal[0] = 1.561479764530789
joint_goal[1] = -1.0873806468016967
joint_goal[2] = 1.2032727274149282
joint_goal[3] = -1.743340078520711#-1.5707
joint_goal[4] = -1.5675044422602604
joint_goal[5] = -3.14042248073452
arm_group.go(joint_goal, wait=True)

hand_group.set_named_target("gripper_open")
hand_group.go(wait=True)


#OBJECT 2 LOCATION

pose_target.position.x = 0.457220879487
pose_target.position.y = 0.223384328948
pose_target.position.z = 0.800
pose_target.orientation.x = -0.304629451702
pose_target.orientation.y = 0.952457338809
pose_target.orientation.z = 0.0048572947131
pose_target.orientation.w = 0.00152367737356
arm_group.set_pose_target(pose_target)
#group.set_path_constraints(upright_constraints)
arm_group.set_planning_time(20)
plan1 = arm_group.plan()
arm_group.go(wait=True)


waypoints = []
wpose = arm_group.get_current_pose().pose
wpose.position.z -=  0.05  # First move up (z)
waypoints.append(copy.deepcopy(wpose))
(plan, fraction) = arm_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0) 
arm_group.execute(plan,wait=True)

group_variable_values = hand_group.get_current_joint_values()
group_variable_values[0] = 0.47
hand_group.set_joint_value_target(group_variable_values)
hand_group.go(wait=True)

waypoints = []
wpose = arm_group.get_current_pose().pose
wpose.position.z +=  0.2  # First move up (z)
waypoints.append(copy.deepcopy(wpose))
(plan, fraction) = arm_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0) 
arm_group.execute(plan,wait=True)

joint_goal = arm_group.get_current_joint_values()
joint_goal[0] = -1.561479764530789
joint_goal[1] = -1.0873806468016967
joint_goal[2] = 1.2032727274149282
joint_goal[3] = -1.743340078520711#-1.5707
joint_goal[4] = -1.5675044422602604
joint_goal[5] = -3.14042248073452
arm_group.go(joint_goal, wait=True)

hand_group.set_named_target("gripper_open")
hand_group.go(wait=True)


#OBJECT 3 LOCATION

upright_constraints = Constraints()
joint_constraint = JointConstraint()
upright_constraints.name = "upright"
joint_constraint.position = 0
joint_constraint.tolerance_above = 0.0
joint_constraint.tolerance_below = 3.14/2#3.1
joint_constraint.weight = 1.0
joint_constraint.joint_name = "shoulder_lift_joint"

pose_target = geometry_msgs.msg.Pose()
pose_target.position.x = 0.562725274795#0.562833913182
pose_target.position.y = -0.240828684149#-0.240600983737
pose_target.position.z = 0.85799
pose_target.orientation.x = 0.0524953115323
pose_target.orientation.y = 0.998612931696
pose_target.orientation.z = -0.0040510154357
pose_target.orientation.w = 0.00021021377887
arm_group.set_pose_target(pose_target)
arm_group.set_path_constraints(upright_constraints)
arm_group.set_planning_time(20)
plan1 = arm_group.plan()
arm_group.go(wait=True)


waypoints = []
wpose = arm_group.get_current_pose().pose
wpose.position.z -=  0.05  # First move up (z)
waypoints.append(copy.deepcopy(wpose))
(plan, fraction) = arm_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0) 
arm_group.execute(plan,wait=True)

group_variable_values = hand_group.get_current_joint_values()
group_variable_values[0] = 0.28
hand_group.set_joint_value_target(group_variable_values)
hand_group.go(wait=True)

waypoints = []
wpose = arm_group.get_current_pose().pose
wpose.position.z +=  0.2  # First move up (z)
waypoints.append(copy.deepcopy(wpose))
(plan, fraction) = arm_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0) 
arm_group.execute(plan,wait=True)

joint_goal = arm_group.get_current_joint_values()
joint_goal[0] = -1.561479764530789
joint_goal[1] = -1.0873806468016967
joint_goal[2] = 1.2032727274149282
joint_goal[3] = -1.743340078520711#-1.5707
joint_goal[4] = -1.5675044422602604
joint_goal[5] = -3.14042248073452
arm_group.go(joint_goal, wait=True)

hand_group.set_named_target("gripper_open")
hand_group.go(wait=True)

moveit_commander.roscpp_shutdown()
