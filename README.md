# RT1_3rd_assignment

# Purpose and Summary
I created final_assignment package in order to learn how to use ROS, Rviz and Gazebo. 

# Tasks
With Rviz and Gazebo, the robot (car) should execute one of the following behaviors.
<br>
①Autonomously reach a x,y coordinate inserted by the user
<br>
②Let the user drive the robot with the keyboard
<br>
③Let the user drive the robot assisting them to avoid collisions
<br>
The simulation with Rviz is shown in the right side.
<img width="269" alt="スクリーンショット 2022-02-18 22 09 54" src="https://user-images.githubusercontent.com/46062824/154762003-3c30aa98-d4f2-421a-b5b6-a0d04b45f81b.png">
<br>
The simulation with Gazebo is shown in the right side.
<img width="320" alt="スクリーンショット 2022-02-18 22 08 10" src="https://user-images.githubusercontent.com/46062824/154762042-a9061a43-5ea8-488f-8271-37bfe2296559.png">

# Menu and Flowchart
The user menu is shown below.
<br>
<img width="490" alt="スクリーンショット 2022-02-19 11 49 15" src="https://user-images.githubusercontent.com/46062824/154797760-4f3cc29e-8640-4c29-8883-e4b961a44fc7.png">

The flowchart of the program is shown below.
<br>

![RT1_2_flowchart](https://user-images.githubusercontent.com/46062824/146186598-33f5dcfd-0092-40b7-878b-9e562c1dd977.JPG)

# Overview of the implementation
There are two nodes I added to the repository provided the professor: "menu" in menu.py and "teleop" in teleop_twist_keyboard.py. I show you the realationships between the nodes in rqt-graph. 

<img width="1388" alt="スクリーンショット 2022-02-18 16 28 46" src="https://user-images.githubusercontent.com/46062824/154717283-b54d1e93-939c-4b1c-a81b-fc4a26d259b2.png">

## "menu" node
### input
- user input from menu.py screen
    - a. driving mode
        - 1: Auto_drive mode
        - 2: Manual_drive mode
        - 3: Assisted_drive mode
        - 4: Cancel the goal
        - 5: Reset car position
        - 0: Exit
    - b. goal coordinates in 1: Auto_drive mode
    - c. command for going back to the menu: if you would like to finish the mode in 2: Manual_drive mode, 3: Assisted_drive mode, you should push p.
- "input_cmd_vel" topic from teleop node: 
In teleop_twist_keyboard.py screen, user can maneuver a car. In 2: Manual_drive mode, 3: Assisted_drive mode, "menu" node receives input_cmd_vel" topic and modifies the velocity depending on the situation related to obstacles around a car.
- "scan" topic from gazebo node: "menu" node receives the information collercted by laser scanner
- "move_base/feedback" and "move_base/goal" topic from move_base node: "menu" node gets the feedback on the current position and the goal the car is aiming at.

### output
- "cmd_vel" topic to gazebo: publish the information on the velocity to gazebo
- "move_base/goal" topic to move_base node: publish the information on the goal to move_base 

## "teleop" node
I used "teleop_twist_keyboard" distributed by ROS org(http://wiki.ros.org/teleop_twist_keyboard). The command menu is shown below. This node publishes "input_cmd_vel" topic to menu node.
<br>
<img width="345" alt="スクリーンショット 2022-02-19 14 47 53" src="https://user-images.githubusercontent.com/46062824/154803549-6dcc6808-88e2-4ed0-ae74-728987021f65.png">

# How to execute
The program can be run in the terminal.
<br>
1. Installing the repository I developed

You should install the repository I developed.

```
$ git clone git@github.com:yusuke817/RT1_3rd_assignment.git
```

2. Installing the package needed

You should install teleop twist keyboard package for driving a car manualy based on user input.

```
$ sudo apt-get install ros-noetic-teleop-twist-keyboard
```
You should install xterm package to show the menu for users.

```
$ sudo apt-get install xterm
```
You should install ros navigation stack.

```
$ sudo apt-get install ros-noetic-navigation
```
3. Preparation

You should run ros itself and compile the program.

```
roscore &
catkin_make
```
4. Running the program

You should run the launch file including simulation_gmapping.launch, move_base.launch, menu node and teleop node. These two nodes are developed and introduced by me.

```
$ roslaunch final_assignment final_assignment.launch
```

# Functions
 1. menu_ui: menu
 2. get_user_input: getting and checking user input and asking for another user input if it is ineffective
 3. display_goal: displaying the goal coordinate
 4. fix_goal: In Auto_drive mode, fixing the goal coordinate
 5. receive_goal: Goal coordinates are stored and subscribed by move_base topic
 6. check_goal: checking whether the goal is reachable or not and notifying arrival 
 7. cancel_process: cancelling the process before reaching the goal
 8. finish_process: finishing the process before reaching the goal
 9. manual_driving: with teleop_twist_keyboard, user can maneuver a car
 10. assisted_driving: in each step, car collects the information on the distance between a car and an object with laserscan topic and decide the speed in assisted mode
 11. actual_vel: with teleop_twist_keyboard, user can change the speed of a car
 12. choose_driving_mode: user can choose the driving mode
 13. main: Initialization, settings about pub/sub and running

# The expalanation about each driving modes

1. "driving_node" in control.cpp.

With pub-sub communications, I implemented the function of automatic driving. After running the node, the robot will drive. In this terminal, the modes and the speeds are displayed continually. There are four modes depending on the movement of the robot: straight, turn right, turn left and decrease. 
   - Driving stragightforward without decreasing the speed when the robot doesn't detect anything in front of it. 
   - Decreasing when the robot detects something in front of it. Also, in this case, depending on the place of the wall, the robot will decide the turning direction
     - Turning right when the robot is close to left wall.
     - Just decreasing when the robot is close to both left and right walls.
     - Turning left when the robot is close to right wall.

2. "speed_server_node" in interaction.cpp

With services, I implemented the function of the change of the speeds and resetting the position. You can give the robot four kinds of commands. You can set the robot's speed from 0.0 to 5.0. You can increase or decrese the speed every 0.5 speeds. 
<br>
  - 'a' for increasing the speed 
  - 'd' for increasing the speed
  - 'r' for resetting the postion and the speed. You can move the robot into its original place anytime.
  - 'f' for terminating the node itself. 

# Result on YouTube
Result is shown in the video below. I recommend you that you should change the resolution into 1080p to read the characters in the terminals.
<br>
[![](https://img.youtube.com/vi/mwPPHWJMLmY/0.jpg)](https://www.youtube.com/watch?v=mwPPHWJMLmY)

# Future work
A car should 
 1. calculate the time of reaching the destination beforehand
 2. register several destinations and move consequently
<br>
The reasons are as follows.
Let's think about the real application. A car moves in the warehouse to carry and put items to the several shelves. In auto driving mode, a car should calculate the time of reaching the destination beforehand not only to check whether or not it is reachable but also to check the amount of consumed fuel. If fuel is not enough to head another destination, it should move back to the starting point to charge the energy. Also, a car should move to the several places until the fuel runs out, I should make a car register several destinations and move consequently. 
