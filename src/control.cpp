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

/*COLORS*/
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

ros::Publisher pub;
float front_th=1.5;
float k_angular=1.0;
float k_linear=0.7;
float default_vel=2.0;
float min_sx;
float min_dx;
float min_front;
float min_front_dx;
float min_front_sx;
float laser [721];
geometry_msgs::Twist my_vel;
ros::ServiceClient client;
ros::ServiceClient client_restart;
second_assignment::KeyboardInput my_input;

float compute_min(int imin, int imax, float ranges[]){

	float min=30;					
	for(int i=imin; i<imax; i++){ //front
		if(ranges[i]<min){
			min=ranges[i];
		}
	}
	return min;	
}

bool interpreter(second_assignment::KeyboardInput::Request &req, second_assignment::KeyboardInput::Response &res){
	
	if(req.input=='a'){
		my_input.response.multiplier+=0.5;
	}
	else if(req.input=='s'){
		my_input.response.multiplier-=0.5;
	}
	else if(req.input=='q'){
		kill(getpid(), SIGINT);
	}
	else if(req.input=='r'){
		my_input.response.multiplier=1.0;
	}
	return true;
}

void Drive(float min_sx, float min_dx, float min_front, float ranges []){

		min_sx=compute_min(620, 720, ranges);
		min_dx=compute_min(0, 100, ranges);
		min_front=compute_min(340, 380 , ranges);
		

		cout<< "\n" BHMAG "Closest laser scan on the right: "<<min_dx<< "\n" RESET;
		cout<< "\n" BHCYN "Closest laser scan on the left: "<<min_sx<< "\n" RESET;
		cout<< "\n" BHBLU "Closest front laser scan: "<<min_front<< "\n" RESET;

	
		if(min_front<front_th){
		cout<< BHRED "Stop, there's a wall in front of me!" RESET "\n";
		
			if(min_sx<0.8*min_dx){
			
				cout<< BHWHT "Turning right..." RESET "\n";
				my_vel.angular.z = -k_angular*my_input.response.multiplier;
				my_vel.linear.x = k_linear*my_input.response.multiplier;
				
			}
			else if(min_dx<0.8*min_sx){
				cout<< BHWHT "Turning left..." RESET "\n";
				my_vel.angular.z = k_angular*my_input.response.multiplier;
				my_vel.linear.x = k_linear*my_input.response.multiplier;
			}
			else {
				my_vel.linear.x = default_vel*my_input.response.multiplier;
			}
			
		}
		
		else{
			cout<< BHGRN "I'll go straight" RESET "\n";
			my_vel.linear.x=default_vel*my_input.response.multiplier;
		}
		pub.publish(my_vel);	
}


void RobotCallback(const sensor_msgs::LaserScan::ConstPtr& msg)

{	
				
		for(int i=0; i<721; i++){
			laser[i]=msg->ranges[i];
		}		
		Drive(min_sx, min_dx, min_front, laser);		
}
        



int main (int argc, char **argv)
{
// Initialize the node, setup the NodeHandle for handling the communication with the ROS //system  
		
	ros::init(argc, argv, "control");  
	ros::NodeHandle nh;


	client =  nh.serviceClient<second_assignment::KeyboardInput>("/keyboard_input");
	client =  nh.serviceClient<std_srvs::Empty>("/reset_positions");
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);  
	my_input.response.multiplier=1.0;
	ros::ServiceServer service= nh.advertiseService("/keyboard_input", interpreter);
	ros::Subscriber sub = nh.subscribe("/base_scan", 1000, RobotCallback);
	ros::spin();
	return 0;
}

