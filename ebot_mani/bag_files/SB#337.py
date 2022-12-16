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
import rospy
import tf
import time
import moveit_commander 
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal


# Initializing moveit_commander 
moveit_commander.roscpp_initialize(sys.argv)

# Initializing rospy node
rospy.init_node('move_python_node',anonymous=True)

# Instantiating a RobotCommander object. This provides information on robot's current joint states.
robot = moveit_commander.RobotCommander() 

# Instantiating a PlanningScene object(provides info on obtaining and updating robot's understanding of the world)
scene = moveit_commander.PlanningSceneInterface()

# Instantiating a MoveGroupCommander object. This object is an interface to a planning group.
arm_group = moveit_commander.MoveGroupCommander("arm")
hand_group= moveit_commander.MoveGroupCommander("gripper")

# Create a DisplayTrajectory ROS publisher (used to display trajectories in Rviz)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path' , moveit_msgs.msg.DisplayTrajectory,queue_size=20)


# Function to move the arm 
# Arguments:- 6D Pose of the location (position and orientation(in qauternion))
#           - Gripper's closing value
#           - Joint value of the robot (waist joint of the robot) which is unique and thus is given individually (we have used joint angles for diversify
#             from the conventional 6D pose to specifiy goal location. The same output was found when feeding moveit with 6D poses.)
def move_arm(pos_x, pos_y, pos_z, or_x, or_y, or_z, or_w, gripper_v, joint_val):

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
    arm_group.set_pose_target(pose_target)

    # Giving the robot a maximum planning time 20 sec to plan its path.
    arm_group.set_planning_time(20)

    # Planning a path to reach the location
    plan1 = arm_group.plan()

    # Executing the path
    arm_group.go(wait=True)


    # STEP 2
    # The robot has to go down by a specific amount to ensure that the object is in between the gripper fingers.
    # Empty list for waypoints to be followed.
    waypoints = []

    # Getting the current pose of the robot, essential as the robot would then iterate cartesian path from the that pose.
    wpose = arm_group.get_current_pose().pose

    # Move down (z) by 5cm
    wpose.position.z -=  0.05  
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm. Thus we specify eef_step as 0.01. 
    # We disable the jump threshold by setting it to 0.0.
    (plan, fraction) = arm_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0) 

    # Sending the plan to the robot for execution.
    arm_group.execute(plan,wait=True)


    # STEP 3
    # Closing the gripper by specific amount 
    # Obtaining the current value of the gripper 
    group_variable_values = hand_group.get_current_joint_values()

    # Moving the gripper to a specific joint value
    group_variable_values[0] = gripper_v

    hand_group.set_joint_value_target(group_variable_values)

    # Sending the values to the robot for execution
    hand_group.go(wait=True)
    rospy.sleep(0.5)

    # STEP 4
    # The robot then proceeds to move up by a specific amount before moving towards goal location.
    # This cartesian path code is as explained above.
    waypoints = []

    wpose = arm_group.get_current_pose().pose

    # Move up (z) by 20 cm
    wpose.position.z +=  0.2  

    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = arm_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0) 
    arm_group.execute(plan,wait=True)


    # STEP 5
    # Move the robot to goal location 
    # We have used joint angles for diversify from the conventional 6D pose to specifiy goal location. 
    # The same output was found when feeding moveit with 6D poses.
    # Obtaining the robot's current joint values.
    joint_goal = arm_group.get_current_joint_values()
    joint_goal[0] = joint_val#1.561479764530789
    joint_goal[1] = -1.0873806468016967
    joint_goal[2] = 1.2032727274149282
    joint_goal[3] = -1.743340078520711
    joint_goal[4] = -1.5675044422602604
    joint_goal[5] = -3.14042248073452

    # Sending the joint set to the arm to execute a path to that specific location.
    arm_group.go(joint_goal, wait=True)


    # STEP 6
    # Opening the gripper to drop the object.
    hand_group.set_named_target("gripper_open")
    hand_group.go(wait=True)



# Main thread

if __name__ == '__main__':
   
    # Creating a Dictionary of locations (position & orientation) 
    Dict = {1:{'pos': [0.57234, -0.01541, 0.791986] , 'orient': [-0.428910594342, -0.903346049818, 0.000156570721622, 0.00126167521046], 'gripper_v': 0.2, 'joint_val': 1.561479764530789}, 
            2:{'pos': [0.457220879487, 0.223384328948, 0.800] , 'orient': [-0.304629451702, 0.952457338809, 0.0048572947131, 0.00152367737356] , 'gripper_v': 0.47, 'joint_val': -1.561479764530789 },  
            3:{'pos': [0.562725274795, -0.240828684149, 0.85799] , 'orient': [0.0524953115323, 0.998612931696, -0.0040510154357, 0.00021021377887] , 'gripper_v': 0.28, 'joint_val': -1.661479764530789 }} 

    for i in range(1,4):
        move_arm(Dict[i]['pos'][0], Dict[i]['pos'][1], Dict[i]['pos'][2], Dict[i]['orient'][0], Dict[i]['orient'][1], Dict[i]['orient'][2], Dict[i]['orient'][3], Dict[i]['gripper_v'], Dict[i]['joint_val'])

    moveit_commander.roscpp_shutdown()

    
