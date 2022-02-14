#! /usr/bin/env python

# import ros stuff
import rospy
import time
import os
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseActionFeedback
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID


# global variable
dis_th = 0.4 # Threshold for reaching the goal 0.2
obj_th = 1.0 # Threshold for avoiding collision 0.7
time_th = rospy.Duration(120) # Max time to let the robot reach the target is 2 minutes
initial_cmd = 'a' # Initialize input for manual driving mode
decided_goal = False # at the beginning, the goal is not decided
vel_msg = Twist() # Define vel_msg 

def menu_ui():
    # menu
    print('Type commands on keyboard to decide the robot driving mode:\n\n')
    print('1: Auto_drive mode: fixting a goal point using (x,y) coordinates\n')
    print('2: Manual_drive mode: driving mode using the keyboard to control the robot\n')
    print('3: Assisted_drive mode: manual driving mode using the keyboard to control the robot\n')
    print('4: Reset robot position\n\n')
    print('0: EXIT THE PROGRAM\n\n')

   #Function that checks the user's input from keyboard and returns it 
   #@return int, user's input

def check_user_input():
    # Getting input from user, it must be integer
    while True:
        try:
            # if input can be converted to an integer it exit from the while
            user_choice = int(input('Command from user: '))
            break
        except:
            print('Could you type an integer number')
    
    return user_choice

   # Function used to print the goal given from user once is fix

def display_goal():
    if decided_goal:
        print('The user decided the goal coordinate\n\n')
        print('Goal:  x = %.2f   y = %.2f\n\n' % (goal_x, goal_y))
        print('Robot is reaching the goal automatically\n')
    else:
        print('Goal has not been fix yet\n\n')

   #Function that fix the objective (point in space) that the robot
   #needs to reach autonomusly

def fix_goal():
    global decided_goal, start_time
    # Clear terminal
    os.system('clear')

    print('1: Auto_drive: fixting a goal point using (x,y) coordinates\n')

    # Ask the goal's x-coordinate to the user, it must be float
    while True:
        try:
            # if input can be converted to a floating number it exit from the while
            in_x = float(input('\nCould you type the x-coordinate for the goal: '))
            break
        except:
            print('Could you type a number?')

    # Ask the goal's y-coordinate to the user, it must be float
    while True:
        try:
            # if input can be converted to a floating number it exit from the while
            in_y = float(input('\nCould you type the y-coordinate for the goal: '))
            break
        except:
            print('Could you type a number?')

    # initialize the goal given by user
    goal_msg = MoveBaseActionGoal()

    goal_msg.goal.target_pose.header.frame_id = "map"
    goal_msg.goal.target_pose.pose.orientation.w = 1

    goal_msg.goal.target_pose.pose.position.x = in_x
    goal_msg.goal.target_pose.pose.position.y = in_y

    # publish goal message and get the time
    pub_goal.publish(goal_msg)
    start_time = rospy.Time.now()

    # goal has been fix
    decided_goal = True


   #Function used to store the goal once it is published
   #@param msg, goal subscribed by move_base topic

def get_goal(msg):
    global goal_x, goal_y
    goal_x = msg.goal.target_pose.pose.position.x
    goal_y = msg.goal.target_pose.pose.position.y

   #Function that tells if the goal has been reached, prints a message and cancel the goal
   #A goal is considered to be unreachable if after five minutes the robot don't arrive to
   #the goal
   #The function is called each time 
   #@param msg, actual goal

def goal_reached(msg):
    global decided_goal
    if decided_goal:
        # Get time
        end_time = rospy.Time.now()
        goal_time = end_time - start_time
        # If time expired, target can not be reached
        if goal_time > time_th:
            cancel_goal()
            print('TIME EXPIRED: target is considered unreachable\n\n')
            time.sleep(1)
            
        # Get robot position in a certain instant
        rob_x = msg.feedback.base_position.pose.position.x
        rob_y = msg.feedback.base_position.pose.position.y

        # See how far the robot is from the goal
        x_dist = rob_x - goal_x
        y_dist = rob_y - goal_y

        # See if it's close enough to be considered goal reached
        if abs(x_dist) < dis_th and abs(y_dist) < dis_th:
            cancel_goal()
            print('GOAL REACHED\n\n') 
            time.sleep(1)

def cancel_goal():
    global decided_goal
    # No goal has been set
    if not decided_goal:
        print('\nThere is no goal to cancel!\n\n')
        time.sleep(2)
     # If there is a goal, cancel it
    else:
        # cancel_msg.id = id
        cancel_msg = GoalID()
        pub_canc.publish(cancel_msg)
        decided_goal = False
        print('\nGoal has been canceled!!!\n\n')
        time.sleep(2)            


   #Function used to let the user acquire the control of the robot
   #The aim is to allow to control the robot's movements using the keyboard

def manual_driving():
    global initial_cmd
    os.system('clear')

    while initial_cmd != 'b':
        # Print the selected manual mode
        if drive_assistance:
            print('\n3: Assisted_drive mode: manual driving mode using the keyboard to control the robot\n\n')
        else:
            print('\n2: Manual_drive mode: driving mode using the keyboard to control the robot\n\n')
        # Get input for exiting from manual mode
        while True:
            try:
                # if waits an input from user
                initial_cmd = input('Press:\nYou should go to to maneuver a car\nb ---> back to main menu\n')
                break
            except:
                print('Could you choose a char?\n')

        # Before exiting the manual mode i need to stop the robot
        if initial_cmd == 'b':
            print('\nEXITING the manual mode\n')
            # fix velocity to zero
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            # Publish the velocity
            pub_vel.publish(vel_msg)
        else:
            print('Could you type: b to go back to main menu\n')
    # fixting initial_cmd with something different from 'b' before exiting
    initial_cmd = 'a' 


#    Function called each time arrives a message from the LaserScan topic
#    If the user asks for driving assistance while it's in manual mode the 
#    funcion gets the min value among a region of the laser scan and take a decision
#    to avoid obstacles
#    If user doesn't sk for assistance, the function does nothing
#    @param msg, array of values given by /scan topic

def assisted_driving(msg):
    global segments, vel_msg
    # If no assistance is required, exit the function
    if not drive_assistance:
        return
    # If assistance is enabled, help the user driving around
    else:
    
        segments = {
            'right':  min(min(msg.ranges[0:143]), 10),
            'fright': min(min(msg.ranges[144:287]), 10),
            'front':  min(min(msg.ranges[288:431]), 10),
            'fleft':  min(min(msg.ranges[432:474]), 10),
            'left':   min(min(msg.ranges[476:714]), 10),
        }
        # Avoid risky situations when the robot is going to collide into walls
        # Ostacle positioned in front of the robot
        if segments['front'] < obj_th:
            # Allow only rotation
            if vel_msg.linear.x > 0 and vel_msg.angular.z == 0:
                # Stop robot's linear velocity
                vel_msg.linear.x = 0

        # Ostacle positioned on the front-right of the robot    
        elif segments['fright'] < obj_th:
            # Allow only rotation on the left
            if vel_msg.linear.x > 0 and vel_msg.angular.z < 0:
                # Stop robot's linear velocity
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0

        # Ostacle positioned on the front-left of the robot
        elif segments['fleft'] < obj_th: 
            # Allow only rotation on the right
            if vel_msg.linear.x > 0 and vel_msg.angular.z > 0:
                # Stop robot's linear velocity
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0

        # Ostacle positioned on the right of the robot
        elif segments['right'] < obj_th:
            # Allow only rotation on the left
            if vel_msg.linear.x == 0 and vel_msg.angular.z < 0:
                # Stop robot's linear velocity
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0

        # Ostacle positioned on the left of the robot
        elif segments['left'] < obj_th: 
            # Allow only rotation on the right
            if vel_msg.linear.x == 0 and vel_msg.angular.z > 0:
                # Stop robot's linear velocity
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
        
        # fix new velocity and publish it
        pub_vel.publish(vel_msg)


#    Function called each time the user uses the teleop keyboard to change the
#    robot's velocity
#    @param 

def fix_user_vel(msg):
    global vel_msg
    # If the robot is not in manual mode, the command given from user is ignored
    if not manu_mode:
    	return
    else:
    	# If the robot is not in drive assistance mode, the velocity is published
    	if not drive_assistance:
    		pub_vel.publish(msg)
    		return
    	# If the robot is in manual mode and the drive assistance is active the message 
    	# given by user is saved and checked from the assisted driving function
    	else:
    		vel_msg.linear.x = msg.linear.x
    		vel_msg.angular.z = msg.angular.z    		
     


#    Function used to decide the behavior of the robot according to 
#    the user input
 
#    @param user_input, user's input

def choose_driving_mode(user_input):
    global manu_mode, drive_assistance
    # selecting the driving mode

    # Autonomus drive aiming at the corrdinate user fixed
    if user_input == 1:
        # aiming at the goal that user wants to reach
        fix_goal()
        if decided_goal:
            os.system('clear')
            # Print on screen the goal
            display_goal()
            # Allow user to see what is the goal
            time.sleep(4)

    # Manual driving mode
    elif user_input == 2:
        # Manual mode activated
        manu_mode = True
        # No assistance
        drive_assistance = False
        manual_driving()
        # Exit from manual mode
        manu_mode = False

    # Assisted manual driving mode
    elif user_input == 3:
        # Manual mode activated
        manu_mode = True
        # Assistance needed
        drive_assistance = True
        manual_driving()
        # fix driving assistance to false
        drive_assistance = False
        # Exit from manual mode
        manu_mode = False
    
    # going back to the initial position
    elif user_input == 4:
        refix_world()
        print('going back to the initial position\n')

    # Exit the loop
    elif user_input == 0:
        # Clear terminal
        os.system('clear')
        # giving the user the information 
        print('finishing the program')
        # killing all of the nodes
        rospy.on_shutdown()

    # Not one of the possible options  
    else:
        # Clear terminal
        os.system('clear')
        print('Invlid command.\n Could you  push 0-4 numbers?:\n\n')
        menu_ui()
        

# ===main function===

def main():
    # initialize global variables
    global decided_goal, drive_assistance, manu_mode, user_input, goal_to_cancel
    global pub_goal, pub_vel, pub_canc, sub_laser, sub_goal, sub_user_vel, sub_robot_pos, refix_world

    # Initialize that the goal has not been fix yet
    decided_goal = False
    # Initialize that there is no need of driving assistence
    drive_assistance = False
    # Initialize that the robot is not in manual mode
    manu_mode = False

    # Initialize the node, fixup the NodeHandle for handling the communication with the ROS system  
    rospy.init_node('main_ui')

    # Create a client to refix the simulation environment
    rospy.wait_for_service('/gazebo/refix_world')
    refix_world = rospy.ServiceProxy('/gazebo/refix_world', Empty)

    # initializing publishers and subscribers
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
    pub_goal = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=100)
    pub_canc = rospy.Publisher('move_base/cancel', GoalID, queue_size=100)
    
    sub_laser = rospy.Subscriber('/scan', LaserScan, assisted_driving)
    sub_goal = rospy.Subscriber('move_base/goal', MoveBaseActionGoal, get_goal)
    sub_user_vel = rospy.Subscriber('/us_cmd_vel', Twist, fix_user_vel)
    sub_robot_pos = rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, goal_reached)

    # Infinite loop until user doesn't press 4 and ros::ok() returns true
    while not rospy.is_shutdown():
        # Print ui
        menu_ui()
        # Get input from user
        user_input = check_user_input()
        # Decide what to do based on user decision
        choose_driving_mode(user_input)


if __name__ == '__main__':
    main()