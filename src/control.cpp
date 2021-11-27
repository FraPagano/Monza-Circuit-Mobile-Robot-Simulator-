/*###LIBRARIES###*/
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Kill.h"
#include "turtlesim/TeleportAbsolute.h"
#include "second_assignment/KeyboardInput.h"
#include "std_srvs/Empty.h"
#include <signal.h>

/*###COLORS###*/
#define RESET "\033[0m"
#define BHBLK "\e[1;90m"
#define BHRED "\e[1;91m"
#define BHGRN "\e[1;92m"
#define BHYEL "\e[1;93m"
#define BHBLU "\e[1;94m"
#define BHMAG "\e[1;95m"
#define BHCYN "\e[1;96m"
#define BHWHT "\e[1;97m"
using namespace std;

/*###PUBLISHER###*/
ros::Publisher pub;

/*###GLOBAL VARIABLES###*/
float front_th=1.5; 		// minimum front distance at which the robot stay from the walls
float front_left_th=1.0;	// minimum front-left distance at which the robot stay from the walls
float front_right_th=1.0;	// minimum front-right distance at wich the robot stay from the walls
float k_angular=15.0; 		// constant angular velocity while the robot is turning
float k_linear=0.4; 		// constant linear velocity while the robot is turning
float default_vel=2.0; 		// default linear velocity while the robot is not facing any wall
float min_left; 		// this variable will contain the minimum distance from the wall computed on the left of the robot  
float min_right; 		// this variable will contain the minimum distance from the wall computed on the right of the robot  
float min_front; 		// this variable will contain the minimum distance from the wall computed on in front of the robot  
float min_front_l;
float min_front_r;
float laser [721]; 		//this float array will contain all the distances in every direction: from 0 (right of the robot) to 721 (left of the robot).

/*###MESSAGES###*/
geometry_msgs::Twist my_vel; //this is the declaration of a geomety_msgs::Twist type message

/*###CUSTOM SERVICE MESSAGE###*/
/*This is the declaration of a second_assignment::KeyboardInput type service message. 
I created this custom service mesage for interpreting the keyboard input from the UI	*/
second_assignment::KeyboardInput my_input; 

/*###CLIENTS###*/
ros::ServiceClient client; //Client that manages the keyboard inputs from UI.
ros::ServiceClient client_restart; //Client that manages the reset request from UI.

/*###FUNCTIONS###*/
float compute_min(int imin, int imax, float ranges[]){
	
	/*This function computes the minimum value among all values in the 'ranges[]' array
	Arguments: 
		1.	i_min (int): smaller index from which the computation should start
		2. 	i_max (int): grater  index in which the computation should end    
		3. 	ranges[] (float): this is the array in which the computation must be done
	Return value:
		1.	min (float): minimum value among all values in the 'ranges[]' array*/
		
	float min=30;					
	for(int i=imin; i<imax; i++){ 
		if(ranges[i]<min){
			min=ranges[i];
		}
	}
	return min;	
}

bool interpreter(second_assignment::KeyboardInput::Request &req, second_assignment::KeyboardInput::Response &res){

	/*This is the function that uses the Service 'service'. It reads the requests from clients and
	change the global variable 'my_input.response.multiplier' that multiplies the velocities so that 
	it makes the robot increase/decrease the velocity. 
	1.	The keyborad key 'a' is used for incrasing the multiplier and therefore also the robot velocity.
	2.	The keyborad key 's' is used for decrasing the multiplier and therefore also the robot velocity.
	3.	The keyborad key 'q' is used for exiting the current node and the user interface node.
	4.	The keyborad key 'r' is used for restarting the robot from its initial position but also for resetting his initial velocity.	 
	
	This is a bool function, it returns true every time that one of the inputs above is received. */
		
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

void Drive(float min_left, float min_right, float min_front, float ranges []){
	
	/*This is the function that drives the robot into the circuit. This function will be called in the Callback function so that
	the instructions will be looped. The implementation logic is quite simple: the minimum distances from the wall to the right, 
	left, and in front of the robot are continuously updated. If the distace in front of the robot is less than 1.5 then a turning method
	was implemented. If the distance on the right is less than the distance on the left, the robot is turning left. And the opposite
	is also true. If no close wall are detected in the front direction from the robot it is simply going straight.
	The velocity managing  was done through the geometry_message::Twist message. The fields 'linear' and 'angular' modify the linear and
	angular velocity of the robot, while the field 'x' 'z' define the orientation axis on which the speed is considered. 
	Finally the message is published.
	Arguments:
		1.	min_left (float): minimum distance from the wall on the left of the robot;
		2.	min_right (float): minimum distance from the wall on the right of the robot;
		3.	min_front (float): minimum distance from the wall in front of the robot.
		4. 	ranges[] (float): array in which the computation takes place.
	Return Values: none	*/
	
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


void RobotCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

		/*This is the Callback function. Here the 'laser' array is filled with the values in the 
		'ranges[]' field of the constant pointer to 'msg' that points to a 'sensor_msgs' object. 'ranges[]' contains 
		information about the distances of the obstacles. Then the Drive(float, float, float, float[]) funciton is called.*/
				
		for(int i=0; i<721; i++){
			laser[i]=msg->ranges[i];
		}		
		
		Drive(min_left, min_right, min_front, laser);		
}
        



int main (int argc, char **argv)
{
	// Initialize the node, setup the NodeHandle for handling the communication with the ROS system  
	ros::init(argc, argv, "control");  
	ros::NodeHandle nh;
	
	// Initialize the clients
	client =  nh.serviceClient<second_assignment::KeyboardInput>("/keyboard_input"); 	// this is for keyboard input from UI node.
	client =  nh.serviceClient<std_srvs::Empty>("/reset_positions");					// this is for restarting the robot position.
	
	// Initialize the server
	ros::ServiceServer service= nh.advertiseService("/keyboard_input", interpreter);
	
	// Define the publisher
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);  // I published the velocity on the 'cmd_vel' topic
	my_input.response.multiplier=1.0; //At first the velocity multiplier must be 1.0
	
	//Initialize and define the subscriber
	ros::Subscriber sub = nh.subscribe("/base_scan", 1000, RobotCallback);
	
	//Loop
	ros::spin();
	
	return 0;
}

