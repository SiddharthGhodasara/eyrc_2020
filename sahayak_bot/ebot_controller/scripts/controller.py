#!/usr/bin/env python

'''
TEAM ID: SB#337

TEAM MEMBERS: SIDDHARTH GHODASARA (TEAM LEADER)
              GAURAV SETHIA
			  HARISH GURAGOL
			  VINEET RANJAN

CODE DESCRIPTION: -> This code is responsible for completing task 1.2, ie. following a given trace (in this case a sine wave) and then
                     reaching a desired goal while avoiding obstacles.
                  -> The motivation behind the structure of this code was state machines, hence the robot changes its state everytime it finishes a task.

                     State machine description: State 0: Wave tracing
                                                State 1: Go to goal
                                                State 2: Obstacle avoidance
                                                        Sub state 1: Find obstacle
                                                        Sub state 2: Turn left by default
                                                        Sub state 3: Follow the obstacle
                                                State 3: Goal reached

                  -> We have used Bug 2 algorithm for obstacle avoidance, as we felt that this would be more robust (much more optimised than bug 1) in tackling most
                     of the obstacles except for a few. Bug 0 would only work for a single obstacle and is known to be one without memory (known direction to goal).


PUBLISHER: "/cmd_vel"             MESSAGE TYPE: geometry_msgs/Twist

SUBSCRIBER: "/ebot/laser/scan"    MESSAGE TYPE: sensor_msgs/LaserScan
            "/odom"               MESSAGE TYPE: nav_msgs/Odometry

GLOBAL VARIABLES: goal      - The desired goal of the robot.
                  pose      - Empty list to store pose of the robot.
                  pub       - Publisher variable to publish from anywhere in the code.
                  min_dist  - The distance at which the robot has to stop before the obstacle.
                  regions   - Storing laser data in a dictionary.
                  equ       - Storing the coefficients of equation of the line Ax + By = C. This is done in order to break out from following the obstacle
                              (by using nearest point from a line logic) and go towards the goal.
                  state     - Initializing state to 0 (State description given above).
		          sub_state - Initializing substate to 0 (Substate description given above).
                  flag      - Counter variable to keep a track of the number of times the robot is close to the goal line.

MEMBER FUNCTIONS : odom_callback   - Obtaining the current pose of the robot.
                   laser_callback  - Obtaining the laser readings of the robot.
                   goto_point      - Going to a desired point.
                   Waypoints       - For generating waypoints for wave tracing.
                   sine_wave       - Main function for wave tracing where in waypoints and goto_point functions are called.
                   get_equation    - Finding the coefficients of the equation of a line constructed from current pose and the desired goal.
                   check_dist      - Checking the distance from the robot's current pose to the constructed line.
                   stop_ebot       - Stopping the robot.
                   take_action     - Consists of all combinations of plausible outcomes for obstacle detection.
                   find_wall       - Makes the robot turn right in a circular manner in order to follow the obstacle.
                   turn_left       - Makes the robot turn left.
                   go_straight     - Makes the robot go straight.
                   follow_obstacle - Main function for obstacle avoidance where in, all the other respective functions are called and substate changes are performed.
                   control_loop    - Main function for the code where in, all the other respective functions are called and state changes are performed.

'''

#Importing Libraries
import math
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


# Initializing global variables
goal = [12.5, 0.0]
pose = []
pub = None
min_dist = 1
equ = None
regions = dict()
state = 0
sub_state = 0
flag = 0


# Odometry Callback function
# Arguments: data (message sent by the publisher)
def odom_callback(data):
    global pose
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]


# Going to a desired point
# Arguments: x,y (point in the obstacle course)
def goto_point(x,y):
    # Proportional gain for yaw of the robot
    Kp = 6
    # Finding out the angle to turn the robot
    angle_to_goal = math.atan2(y - pose[1], x - pose[0])
    # Instantiating a geometry_message/Twist object
    msg = Twist()

    while (abs(pose[0] - x) >= 0.005 and abs(pose[1] - y) >= 0.001):
        # If robot detects an obstacle in front
        if regions['front'] < min_dist:
            global state
            #Changing the state to 2
            state = 2
            return
        else:
            theta = pose[2]
            # Finding the yaw error
            yaw_error =  angle_to_goal - theta
            yaw = Kp * yaw_error

            #Checking if orientation is correct
            if abs(yaw_error) >= 0.01:
                msg.linear.x = 0.08
                msg.angular.z = yaw
            else:
                msg.linear.x = 0.3
                msg.angular.z = 0.0
            pub.publish(msg)

    global state
    state = 3


# Function to generate way-points
# Arguments: t (x values in the form of waypoints)
def Waypoints(t):
    x = t
    y = 2*np.sin(t)*np.sin(t/2)
    return [x,y]


# Main function for wave tracing where in waypoints and goto_point functions are called
# Arguments: None
def sine_wave():
    # Waypoint generated every 0.2 change in x
    for i in np.arange(0, 6.6, 0.2):
        # Getting the goal point
        [x,y] = Waypoints(i)
        print("Got waypoint ", x, y)
        goto_point(x,y)
    print("Sine wave Done!")


# Function to find the coefficients of the equation of a line constructed from current pose and the desired goal
# Arguments: curr_point, goal (current point of the robot and desired goal)
def get_equation(curr_point, goal):
    # a = y2 - y1
    A = goal[1] - curr_point[1]
    # b = x2 - x1
    B = goal[0] - curr_point[0]
    # c = x1*y2 - y1*x2
    C = curr_point[0]*goal[1] - curr_point[1]*goal[0]
    return [A,B,C]


# Function to check the distance from the robot's current pose to the constructed imaginary line
# Arguments: point (current pose of the robot)
def check_dist(point):
    # Checking for errors
    try:
		x = point[0]
		y = point[1]
        # distance = (ax1 + by1 + c)/sqrt(a^2 + b^2)
		dist = ((equ[0]*x) + (equ[1]*y) + equ[2])/(equ[0]**2 + equ[1]**2)**0.5
		return dist
    
    except:
		return 0


# Laser Callback function
# Arguments: msg (message sent by the publisher)
def laser_callback(msg):
    global regions
    regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }


# Function to stop the robot
# Arguments: None
def stop_ebot():
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    pub.publish(msg)

# Function to elucidate all combinations of plausible outcomes for obstacle detection
# Arguments: None
def take_action():
    global regions
    global sub_state
    global state
    global flag


    # Following the obstacle until robots comes near the goal line twice
    if flag < 2:

        # Changing the sub states according to the data from Laser
        if regions['front'] > min_dist and regions['fleft'] > min_dist and regions['fright'] > min_dist:
            sub_state = 0
        elif regions['front'] < min_dist and regions['fleft'] > min_dist and regions['fright'] > min_dist:
            sub_state = 1
        elif regions['front'] > min_dist and regions['fleft'] > min_dist and regions['fright'] < min_dist:
            sub_state = 2
        elif regions['front'] > min_dist and regions['fleft'] < min_dist and regions['fright'] > min_dist:
            sub_state = 0
        elif regions['front'] < min_dist and regions['fleft'] > min_dist and regions['fright'] < min_dist:
            sub_state = 1
        elif regions['front'] < min_dist and regions['fleft'] < min_dist and regions['fright'] > min_dist:
            sub_state = 1
        elif regions['front'] < min_dist and regions['fleft'] < min_dist and regions['fright'] < min_dist:
            sub_state = 1
        elif regions['front'] > min_dist and regions['fleft'] < min_dist and regions['fright'] < min_dist:
            sub_state = 0
        else:
            rospy.loginfo(regions)

        # Checking when the robot comes close to the goal line for 1st time
        if check_dist(pose) > 0.6 and flag == 0:
            print("Changed flag to 1")
            flag = 1

        # Checking when the robot comes close to the goal line for 2nd time
        # Stop following the obstacle and move towards the goal
        if check_dist(pose) <= 0.5 and flag == 1:
            print("Point reached stoppping ebot")
            state = 1
            sub_state = 3
            flag = 2


# Function to turn the robot right in a circular manner in order to follow the obstacle
# Arguments: None
def find_wall():
    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = -0.3
    pub.publish(msg)


# Funtion to turn the robot left
# Arguments: None
def turn_left():
    msg = Twist()
    msg.angular.z = 0.3
    pub.publish(msg)


# Function to make the robot go straight in order to follow obstacle
# Arguments: None
def go_straight():
    msg = Twist()
    msg.linear.x = 0.3
    pub.publish(msg)


# Main function for obstacle avoidance where in, substate changes are performed
# Arguments: None
def follow_obstacle():
    global equ
    equ = get_equation([pose[0], pose[1]], goal)
    global sub_state
    rate = rospy.Rate(10)
    # Sub state machine, until it comes close to goal line
    while state == 2:
        take_action()
        if sub_state == 0:
            find_wall()
        elif sub_state == 1:
            turn_left()
        elif sub_state == 2:
            go_straight()
        elif sub_state == 3:
            stop_ebot()
        else:
            rospy.logerr('Unknown state!')

        rate.sleep()


# Controller Function
# Arguments: None
def control_loop():
    # Initializing the node
    rospy.init_node('ebot_controller')
    global pub
    # Defining velocity publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # Defining laser scan subscriber
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    # Defining odometry subscriber
    rospy.Subscriber('/odom', Odometry, odom_callback)

    rate = rospy.Rate(10)

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    rospy.sleep(1)

    global state

    # State machine
    while not rospy.is_shutdown():
        if(state==0):
            print("Sine wave")
            sine_wave()
            state = 1
        elif(state==1):
            print("Goto point")
            goto_point(goal[0], goal[1])
            state = 2
        elif state == 2:
            print("Follow obstacle")
            follow_obstacle()
        elif state == 3:
            print("Location reached")
        else:
            break
        rate.sleep()
        stop_ebot()


#Main Thread
if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
