# RT1_3rd_assignment

# Purpose and Summary
I created second_assignment package in order to learn how to use ROS expecially in Pub/Sub communications and services.

# Tasks
①Robot can drive in the circuit automatically without crashing on the walls.
<br>
②An user can change the speed of the robot.

# Explanation about the situation and implementation
The blue dot in the circuit is the robot controlled in this assignment.
<br>
There are two nodes I developped: driving_node in control.cpp and speed_server_node in interaction.cpp.
<br>
1. Driving_node is developped for controlling the movement of the robot.
2. Speed_server_node is developped for changing the speed of the robot and resetting the position of the robot depending on the user inputs.

<img width="772" alt="ros_stage" src="https://user-images.githubusercontent.com/46062824/145958992-266706c8-2900-4d03-b4f2-cdf08531265c.png">

# Flowchart
The flowchart of the program is shown below.
<br>

![RT1_2_flowchart](https://user-images.githubusercontent.com/46062824/146186598-33f5dcfd-0092-40b7-878b-9e562c1dd977.JPG)

# How to execute
The program can be run with 4 terminals shown below: main, stage_ros, the one for driving_node and the one for speed_server_node.
<img width="1402" alt="terminal×4" src="https://user-images.githubusercontent.com/46062824/146006462-8eb17c5e-ea8a-4541-b6eb-e9bb1c22b2a5.png">
<br>
1. Preparation

You should run ros itself and compile the program.

```
roscore &
catkin_make
```
2. Running

You should run the program with 3 nodes in each terminals respectively.

```
rosrun stage_ros stageros $(rospack find second_assignment)/world/my_world.world
```

```
rosrun second_assignment driving_node
```

```
rosrun second_assignment speed_server_node
```

# The expalanation about each nodes for operation

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

# How to develop each files

This program has two source codes: control.cpp and interaction.cpp.

## 1. control.cpp including "driving_node".

There are four functions in control.cpp: ①sensor function, ②ScanCallback function, ③SpeedService function and ④main function.
<br>
①sensor function is for calculating the shortest distance in the array values collected with laser sensors.
<br>
②ScanCallback function is composed of three parts: ❶collection of the minimum distance in each three sections: right, front and left ❷segmentation the cases depending on the minimum distance ❸change of the linear and angular speeds based on the segmentation.

❶collection code is shown below. This program collects the distance in all directions. The direction is divided into 3 parts: right with from 0th to 100th sensor, front with from 300th to 390th sensor and left with 620th to 720th sensor. 
```
	int ranges = msg->ranges.size();
	float s[ranges];
	for(int i =0; i<ranges; i++)
	{
		s[i]=msg->ranges[i];
	}
	// to get the information on the distance in each sectons: right, front and left.
	float right[101];
	float front[91];
	float left[101];
	
	// right section
	for (int k = 0; k <= 100; k++)
	{
		right[k] = s[k];
	}
	// front section
	for (int k = 300; k <= 390; k++)
	{
		front[k-300] = s[k];
	} 
	// left section
	for (int k = 620; k <= 720; k++)
	{
		left[k-620] = s[k];
	}
```

❷segmentation and ❸change part are implemented with minimum distance derived in ❶collection and the threshold I set. The one part of the code is shown below.

```
float dist_th = 1.5;// the threshold of the distance between the robot and walls

	if(sensor(front) <= dist_th)
	{
		// left wall is close
		if(sensor(right)-sensor(left)> 0)
		{
			ROS_INFO("Turn right");
			vel.linear.x = 0.5;
			vel.angular.z = -1.0;
		}
		
	}
	
	// observing the linear velocity 
	ROS_INFO("%f", vel.linear.x);
	// publishing the velocity 
	pub.publish(vel);

```

③SpeedService function is composed of two parts: ❶changing the speed part and ❷ resetting the postion and speed part. ❶ If "res.input" is not -1000 which means that user did not give the reset command, the information on the speed is updated with speed.srv service. ❷ When user gives the reset command, the ros service call related to reset the position is called and resets the position and the speed. The part of the code is shown below.

```
bool SpeedService(second_assignment::speed::Request &req, second_assignment::speed::Response &res) {
	ROS_INFO("serviceget");
	// except "reset", the increment of the speed is updated
	if (req.input != -1000.0){
	plus = req.input;
	res.output = plus;
	ROS_INFO("change");
	}
	// in reset, resetting the position and the speed is done
	else{
	// resetting the position is done with the service call
	ros::service::call("/reset_positions", res_server);
	ROS_INFO("reset");
	// resetting the speed is done as well
	plus = 0;
	original=0.0;
	} 
	return true;
}
```

④In main function, subscibing base_scan topics and advertising the "speed" services to the client are done to run this node. The part of the code is shown below.
```
int main(int argc, char **argv)
{
	ROS_INFO("main");
	
	//initializing the node and setting up the node hundle
    	ros::init(argc, argv, "driving_node");
	ros::NodeHandle nh;	
	//subscribing /base_scan topics
	ros::Subscriber sub = nh.subscribe("/base_scan", 1, ScanCallback);
	pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
	// advertising the "speed" services to the client
	ros::ServiceServer service =  nh.advertiseService("/speed", SpeedService);
	ros::spin();
	return 0;
}
```

## 2. interaction.cpp including "speed_server_node" 

There is only one main function in interaction.cpp. There are three main processes: ①getting user input, ②storing the user input into the request of the server and ③sending a request to the server as a client. The one part of the code is shown below.

```
	while(1){
	//guide for users
	std::cout << "Increase: a or Decrease: d or Reset: z or Finish: f?\n";
	//getting the user input
	std::cin >> s;
	
	// when users want to increase the speed
			if(s == 'a'){
				//liniting the range of the speed
				if(original > -100.0 && original <=3.5){
				// increasing the speed
				original += 0.5;
				ROS_INFO("increase");
				//storing the user input into the request of the server
				speed_srv.request.input = original;
				//sending a request to the server
				client.call(speed_srv);
				     }
   		}
```

# rqt-graph
I made the folder called "second_assignment." I show you the realationships between the nodes in rqt-graph. Driving node sends the commands on the velocity to stageros node. Also, driving node can get the updated velocity based on user input from speed server node with the speed.srv.
<img width="1382" alt="RT2_rqt" src="https://user-images.githubusercontent.com/46062824/146010312-9e4327dd-0b36-4c1c-b1f4-5632a9c72ed9.png">

# Result on YouTube
Result is shown in the video below. I recommend you that you should change the resolution into 1080p to read the characters in the terminals.
<br>
[![](https://img.youtube.com/vi/FFKL9n6XOKk/0.jpg)](https://www.youtube.com/watch?v=FFKL9n6XOKk)

# Future work
In this assignment, I set the limitation of the robot since the robot will crash against the wall when the speed is too big. With PID control, the quality of control will improve and robot will drive faster than now.
