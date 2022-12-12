#!/usr/bin/env python

'''
TEAM ID: SB#337

TEAM MEMBERS: SIDDHARTH GHODASARA (TEAM LEADER)
              GAURAV SETHIA
			  HARISH GURAGOL
			  VINEET RANJAN
	      
CODE DESCRIPTION: -> This code is responsible for completing task 4, ie. moving a UR5 robotic arm using Moveit to pick and place 3 objects(coke, battery, glue)
                     while avoiding obstacles whilst using object poses published by find_object_2d.

                  -> The following steps are taken to ensure the robot performs this task:
                     STEP 1: Detect the object and obtain its x,y,z values through the find_object_2d package using sift descriptor. 
                     STEP 2: Move the robot to go near the object.
                     STEP 3: The robot then moves suc that the object is placed between its fingers.
                     STEP 4: It then closes the gripper thus grasping the object.
                     STEP 5: Moves up whilst holding the object.
                     STEP 6: Moves to the drop location.
                     STEP 7: Drops the object.  
                     
                     The above steps take place for each object.

PUBLISHER: "/move_group/display_planned_path"     MESSAGE TYPE: moveit_msgs.DisplayTrajectory
           "/detection_info"     MESSAGE TYPE: object_msgs.ObjectPose
 
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
from object_msgs.msg import ObjectPose
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

# Move the arm to initial known location.
arm_group.set_named_target("init_pose")

# Sending the arm to execute a path to that specific location.
arm_group.go(wait=True)

# Function to move the arm 
# Arguments:- 6D Pose of the location (position and orientation(in qauternion))
#           - Gripper's closing value
def move_arm(pos_x, pos_y, pos_z, gripper_v):

    # Move the robot to hover above the object.
    # Giving the robot a 6D pose on the object location.
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = pos_x
    pose_target.position.y = pos_y - 0.2
    pose_target.position.z = pos_z + 0.07  
    
    pose_target.orientation.x = -0.0267950343469
    pose_target.orientation.y = -0.883912377391
    pose_target.orientation.z = -0.466875953503
    pose_target.orientation.w = 0.00278913429132

    # Specifying the target pose to the planning group of the robot.
    arm_group.set_pose_target(pose_target)

    # Giving the robot a maximum planning time 20 sec to plan its path.
    arm_group.set_planning_time(20)

    # Planning a path to reach the location
    plan1 = arm_group.plan()

    # Executing the path
    arm_group.go(wait=True)


    # The robot has to go down by a specific amount to ensure that the object is in between the gripper fingers.
    # Empty list for waypoints to be followed.
    waypoints = []

    # Getting the current pose of the robot, essential as the robot would then iterate cartesian path from the that pose.
    wpose = arm_group.get_current_pose().pose

    # Move sideways (y) by 10cm
    wpose.position.y +=  0.105  
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm. Thus we specify eef_step as 0.01. 
    # We disable the jump threshold by setting it to 0.0.
    (plan, fraction) = arm_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0) 

    # Sending the plan to the robot for execution.
    arm_group.execute(plan,wait=True)

    # Closing the gripper by specific amount 
    # Obtaining the current value of the gripper 
    group_variable_values = hand_group.get_current_joint_values()

    # Moving the gripper to a specific joint value
    group_variable_values[0] = gripper_v

    hand_group.set_joint_value_target(group_variable_values)

    # Sending the values to the robot for execution
    hand_group.go(wait=True)


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
    
    arm_group.set_named_target("init_pose")

    # Sending the arm to execute a path to that specific location.
    arm_group.go(wait=True)

    # STEP 5
    # Move the robot to drop location 
    # Obtaining the robot's current joint values.
    arm_group.set_named_target("drop_pose")

    # Sending the joint set to the arm to execute a path to that specific location.
    arm_group.go(wait=True)

    # STEP 6
    # Opening the gripper to drop the object.
    hand_group.set_named_target("gripper_open")
    hand_group.go(wait=True)


    arm_group.set_named_target("init_pose")
    # Sending the arm to execute a path to that specific location.
    arm_group.go(wait=True)
    


# Main thread
if __name__ == '__main__':

    # Initialise a publisher topic detectioon info of type ObjectPose
    pub = rospy.Publisher('/detection_info', ObjectPose, queue_size=10)

    # Instantiating objects of ObjectPose type for each corresponding items (coke_can, battery, glue_box)
    coke_can = ObjectPose()
    battery = ObjectPose()
    glue_box = ObjectPose()

    # Listen to all trasnforms
    transform = tf.TransformListener()
    # Give it some time to detect it
    rospy.sleep(1)

    # Store the tranformed values in respective variables
    (trans_coke,rot_coke) = transform.lookupTransform("ebot_base", "object_13", rospy.Time())
    (trans_battery,rot_battery) = transform.lookupTransform("ebot_base", "object_14", rospy.Time())
    (trans_glue,rot_glue) = transform.lookupTransform("ebot_base", "object_15", rospy.Time())

    
    # Publishing detected object's postion
    (translation,rotation) = transform.lookupTransform("base_link", "object_13", rospy.Time())
    (translation,rotation) = transform.lookupTransform("base_link", "object_14", rospy.Time())
    (translation,rotation) = transform.lookupTransform("base_link", "object_15", rospy.Time())

    # Publish x,y,z of detected coke_can with respect to base_link
    coke_can.pose.header.frame_id = "base_link"
    coke_can.name = "coke_can"
    coke_can.pose.pose.position.x = translation[0]
    coke_can.pose.pose.position.y = translation[1]
    coke_can.pose.pose.position.z = translation[2]
    pub.publish(coke_can) 

    # Publish x,y,z of detected battery with respect to base_link
    battery.pose.header.frame_id = "base_link"
    battery.name = "battery"
    battery.pose.pose.position.x = translation[0]
    battery.pose.pose.position.y = translation[1]
    battery.pose.pose.position.z = translation[2]
    pub.publish(battery)

    # Publish x,y,z of detected glue with respect to base_link
    glue_box.pose.header.frame_id = "base_link"
    glue_box.name = "glue"
    glue_box.pose.pose.position.x = translation[0]
    glue_box.pose.pose.position.y = translation[1]
    glue_box.pose.pose.position.z = translation[2]
    pub.publish(glue_box)

    # Creating a Dictionary of locations (position & gripper values) 
    Dict = {1:{'pos': [trans_coke[0], trans_coke[1], trans_coke[2]], 'gripper_v': 0.265}, 
            2:{'pos': [trans_battery[0], trans_battery[1], trans_battery[2]], 'gripper_v': 0.29},  
            3:{'pos': [trans_glue[0], trans_glue[1], trans_glue[2]], 'gripper_v': 0.325}} 

    # Calling the move_arm function three times
    for i in range(1,4):
        move_arm(Dict[i]['pos'][0], Dict[i]['pos'][1], Dict[i]['pos'][2], Dict[i]['gripper_v'])

    moveit_commander.roscpp_shutdown()

    