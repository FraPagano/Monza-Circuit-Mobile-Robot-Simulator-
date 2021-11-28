## Research Track 1, second assignment

================================

### Introduction

--------------------------------

The second assignment of the [Research Track 1](https://unige.it/en/off.f/2021/ins/51201.html?codcla=10635) class is about a robot simulator using the ROS framework and the C++ programing language. The robot essentially is represented by a blue cube and we were supposed to make the robot navigate into a predefinite maze that is the famous Monza's circuit.  Moreover, a User Interface was requested and it should constantly wait for an input for the user, which can either ask to increment or decrement the velocity, or to put the robot in the initial position. Therefore, two different nodes were necessary: 

 1. The controlling node, that I called **control.cpp**
 2. The UI node, that I called **ui.cpp**

Here's some pictures that show the simulation enviroment provided us by profesor [Carmine Recchiuto](https://github.com/CarmineD8):
<p align="center">
 <img src="https://github.com/FraPagano/RT_Assignment_2/blob/main/Videos,%20gifs%20%20and%20images/Robot.JPG" height=320 width=380>
</p>
<p align="center">
 <img src="https://github.com/FraPagano/RT_Assignment_2/blob/main/Videos,%20gifs%20%20and%20images/Circuit.JPG" height=320 width=380>
</p>

The two nodes I created are quite simple: in the controller node the idea is to constantly check for data from laser sensors which the robot is equipped with and make turning decisions on the base of such data. The default velocities were set by implementing a publish–subscribe messaging pattern. On the other hand, the UI node waits for inputs from the user and by using a custom Service it modifies the robot velocity depending on the pressed key. Some additional features were added on the UI node. 

The greaest issues that I faced with during the implementation of the project were:

 - Become familiar with the ROS framework;

 - designing a turn decision method;

 - create a code that implements such turn decision method;

 - find all the correct parameters (i.e. linear and angular velocity,  threshold values, etc..) in such a way as not to make the robot navigate too fast or too slow. Such parameters are important because by choosing  wrong values the robot navigation results risky, or that the the robot should be likely to bump into the walls.

### Code description
---------------------------
Some important global object were instantiated such as:

 1. A `geometry_msgs::Twist my_vel` message used for setting both the robot's linear and angular default velocities;
 2. A custom service message `second_assignment::KeyboardInput my_input` whose `request` field is a *char* type and `response` field is a *float* type. This custom service message has been on purpose designed for the user keyboard's inputs;
 3. A `ros::ServiceClient` for the velocity changes, handled by the UI node; 
 4. Another `ros::ServiceClient` for the restart request, handled by the UI node.

#### Controlling node
The controlling node is the one that set  both the robot's linear and angular velocities and elaborates data from laser sensors. Here, a turn decison method was implemented but also a Server that collects the Client (in the UI node) requests.
The functions that I created are:

 - `bool interpreter(second_assignment::KeyboardInput::Request &req,
   second_assignment::KeyboardInput::Response &res)`:  This is the server function that reads the requests from clients and changes the global variable `my_input.response.multiplier` that multiplies the default velocities so that it makes the robot's velocity increase/decrease. In particular,
		1.	The keyborad key 'a' is used for increasing the multiplier and therefore also the robot velocity.
		2.	The keyborad key 's' is used for decreasing the multiplier and therefore also the robot velocity.
		3.	The keyborad key 'q' is used for terminating the processes.
		4.	The keyborad key 'r' is used for restarting the robot from its initial position but also for resetting his initial velocity.	 
This is a bool function, it returns `true` every time that one of the inputs above is received.
Here's the code:
```c++
bool interpreter(second_assignment::KeyboardInput::Request &req, second_assignment::KeyboardInput::Response &res)
{
	if(req.input=='a'){
		my_input.response.multiplier+=0.5;
	}
	else if(req.input=='s'){
		my_input.response.multiplier-=0.5;
	}
	else if(req.input=='q'){
		kill(getpid(), SIGINT); // Kill the current process. (I included the <signnal.h> library!)
	}
	else if(req.input=='r'){
		my_input.response.multiplier = 1.0;
	}
	return true;
}
```
- `float compute_min(int imin, int imax, float ranges[])`: This function computes the minimum value among all values in the `ranges[]` array. The arguments are: 
		1.	`i_min` (int): smaller index from which the computation should start
		2. 	`i_max` (int): grater  index in which the computation should end    
		3. 	`ranges[]` (float): this is the array in which the computation must be done
	
It returns the minimum value among all values in the `ranges[]` array. 
Here's the code:
``` c++
float compute_min(int imin, int imax, float ranges[]){
	float min=30;					
	for(int i=imin; i<imax; i++){ 
		if(ranges[i]<min){
			min=ranges[i];
		}
	}
	return min;	
}
```

 - `void Drive(float min_left, float min_right, float min_front, float ranges [])`: This is the function that drives the robot into the circuit. This function will be called in the Callback function so that the instructions will be looped. The implementation logic is quite simple: the minimum distances from the wall to the right, left, and in front of the robot are continuously updated. If the distace in front of the robot is less than 1.5 then a turning method is activated. If no walls closer than 1.5 are detected in the front direction the robot is simply going straight. In order to read all the data from the laser sensor which the robot is equipped with I implemented a subscriber to the `"/base_scan"` topic. This topic uses a message of type [`sensor_msgs::LaserScan`](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html) whose field called `ranges`  is an array of 721 elements. Such array provide us the distances from the walls in every direction. Element 0 give us the distance from the wall on the right side and the 721st element give us the distance on the left side. The logic is quite simple: if the distance on the right is less than the distance on the left, the robot is turning left. And the opposite is also true. The array's checked spans  are:  
	 - Left side, corresponding to the 0-100 array span.
	 - Right side, corresponding to the 620-721 array span.
	 - Front direction, corresponding to the 300-420 array span.
	 - Front-left side, corresponding to the 450-510 array span.
	 - Front-right side, corresponding to the 170-230 array span
	 
	Thanks to the `compute_min(int imin, int imax, float ranges[])` function I could extrapolate the lowest distances among each span and therefore I could also let the robot make decisions based on the minimum distances. 
The velocity managing  was done through the `geometry_message::Twist` message. The fields *linear* and *angular* have been used to modify the linear and angular velocity of the robot, while the fields *x* and *z* define orientation axis on which the speed is considered. Finally, the message is published.
	Arguments:
			1.	`min_left` (float): minimum distance from the wall on the left of the robot;
			2.	`min_right` (float): minimum distance from the wall on the right of the robot;
			3.	`min_front` (float): minimum distance from the wall in front of the robot.
			4. 	`ranges[]` (float): array in which the computation takes place.
			
	There are no return values for this funciton.
	Here's the code:
```c++
void Drive(float min_left, float min_right, float min_front, float ranges []){
	
	min_left=compute_min(620, 720, ranges);
	min_right=compute_min(0, 100, ranges);
	min_front=compute_min(300, 420 , ranges);
	min_front_l=compute_min(450, 510, ranges);
	min_front_r=compute_min(170, 230, ranges);
		
	cout<< "\n" BHMAG "Closest laser scan on the right: "<<min_right<< "\n" RESET;
	cout<< BHCYN "Closest laser scan on the left: "<<min_left<< "\n" RESET;
	cout<< BHBLU "Closest front laser scan: "<<min_front<< "\n" RESET;

	if(min_front<front_th){
		
		cout<< BHRED "Stop, there's a wall in front of me!" RESET "\n";
		if(min_left<0.8*min_right){
		
			cout<< BHWHT "Turning right..." RESET "\n";
			my_vel.angular.z = -k_angular;
			my_vel.linear.x = k_linear*my_input.response.multiplier;
		}
		else if(min_right<0.8*min_left){
			
			cout<< BHWHT "Turning left..." RESET "\n";
			my_vel.angular.z = k_angular;
			my_vel.linear.x = k_linear*my_input.response.multiplier;
		}
		else if(min_front_l<front_left_th){
			cout<< BHWHT "Turning a little bit left..." RESET "\n";
			my_vel.angular.z = k_angular;
			my_vel.linear.x = k_linear;
		}
		else if(min_front_r<front_right_th){
			cout<< BHWHT "Turning a little bit right..." RESET "\n";
			my_vel.angular.z = -k_angular;
			my_vel.linear.x = k_linear;
		}
		else {
			
			my_vel.linear.x = default_vel*my_input.response.multiplier;
			my_vel.angular.z = 0.0;
		}
	}
	else{
		
		cout<< BHGRN "I'll go straight" RESET "\n";
		my_vel.linear.x=default_vel*my_input.response.multiplier;
		my_vel.angular.z = 0.0;
	}
	cout<< BHWHT "The robot current linear velocity is "<<my_input.response.multiplier<<" times the default velocity\n" RESET;
	pub.publish(my_vel);	
}
```
#### UI node


###  Installing and running 
----------------------
Here's some useful informations regarding running the simulator.
First of all, 

### Flowchart

--------------------------------

For a more precise description of what my code does you can consult the following flowchart, created with [Lucidchart](https://www.lucidchart.com/pages/it/landing?utm_source=google&utm_medium=cpc&utm_campaign=_chart_it_allcountries_mixed_search_brand_bmm_&km_CPC_CampaignId=9589672283&km_CPC_AdGroupID=99331286392&km_CPC_Keyword=%2Blucidcharts&km_CPC_MatchType=b&km_CPC_ExtensionID=&km_CPC_Network=g&km_CPC_AdPosition=&km_CPC_Creative=424699413299&km_CPC_TargetID=kwd-334618660008&km_CPC_Country=1008337&km_CPC_Device=c&km_CPC_placement=&km_CPC_target=&mkwid=sKwFuAgHb_pcrid_424699413299_pkw_%2Blucidcharts_pmt_b_pdv_c_slid__pgrid_99331286392_ptaid_kwd-334618660008_&gclid=CjwKCAjw5c6LBhBdEiwAP9ejG86DblinG5ivYRvMmKSvI8Dl7as9i2oINlmgqIDoj0gpLX6WfnCenRoCxxQQAvD_BwE)

### Results
--------------------------------

The final result is that the robot correctly runs around the circuit and, despite there are some things that could be improved in the future, I am satisfied with the work that I've done specially because that was my first approach with the ROS framework. The whole work was carried out together with my friends and uni colleagues.

In order to make you understand how my code works, I recorded this video:


### Possible Improvements

--------------------------------



