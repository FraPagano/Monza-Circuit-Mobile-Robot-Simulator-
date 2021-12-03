#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "second_assignment/KeyboardInput.h"
#include "std_srvs/Empty.h"

/*###LIBRARIES###*/
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

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

/*###GLOBAL VARIABLES###*/
char command;
char exit_command;

/*###CLIENTS###*/
ros::ServiceClient client; 
ros::ServiceClient client_restart; 

/*###CUSTOM SERVICE MESSAGE###*/
/*This is the declaration of a second_assignment::KeyboardInput type service message. 
I created this custom service mesage for interpreting the keyboard inputs.	*/
second_assignment::KeyboardInput my_input;

/*###SERVICE MESSAGE###*/
std_srvs::Empty restart_srv; //this service message is used for resetting the robot's position


int main(int argc, char **argv)
{
	cout<< "\n" BHBLK "################# USER INTERFACE #################" RESET "\n";
	cout<< BHGRN "Press 'a' for increasing the robot velocity!" RESET"\n"; 
	cout<< BHBLU "Press 's' for decreasing the robot velocity!" RESET"\n"; 
	cout<< BHCYN "Press 'r' for resetting the robot initial position and velocity!" RESET"\n"; 
	cout<< BHRED "Press 'q' for quitting the applicaiton!" RESET"\n";
	
	// Initialize the node, setup the NodeHandle for handling the communication with the ROS system  
	ros::init(argc, argv, "ui");
	ros::NodeHandle n;
	
	//Define the clients
	client =  n.serviceClient<second_assignment::KeyboardInput>("/keyboard_input");
	client_restart =  n.serviceClient<std_srvs::Empty>("/reset_positions");
	
	//Waiting for inputs...
	while(ros :: ok()) {
		cin>>command;
		
		if(command=='a'){
			my_input.request.input='a'; 				//Setting the request
			client.waitForExistence(); 				//Waiting for the service to be advertised and available
			client.call(my_input); 					//Call the service
			cout<< BHGRN "Increasing..." RESET "\n";
		}
		else if(command=='s'){
			my_input.request.input='s'; 				//Setting the request
			client.waitForExistence();				//Waiting for the service to be advertised and available
			client.call(my_input); 					//Call the service
			cout<< BHBLU "Decreasing..." RESET "\n";
		}
		else if(command=='r'){
			client_restart.waitForExistence();			//Waiting for the service to be advertised and available
			client_restart.call(restart_srv); 			//Call the service
			my_input.request.input='r'; 				//Setting the request
			client.waitForExistence();				//Waiting for the service to be advertised and available
			client.call(my_input); 					//Call the service
			cout<< BHCYN "Restarting..." RESET "\n";
		}
		else if(command=='q'){
			cout<< BHRED "Are you sure you want to quit? 'y' for Yes, 'n' for No" RESET "\n";
			cin>>exit_command;		
			if(exit_command=='y'){
				my_input.request.input='q'; 			//Setting the request
				client.call(my_input); 				//Call the service
				cout<< BHRED "Exiting..." RESET "\n";
				return 0;					//Terminating the current process
			}
			else if(exit_command=='n'){
				cout<< BHGRN "Okay, let's continue!" RESET "\n";
			}
			else{
				cout<< BHRED "Press 'y' for Yes, 'n' for No" RESET "\n";
			}
		}
		else{ // A not allowed command was pressed
			cout<< BHRED "This command is not allowed, please use the commands above!" RESET "\n";
		}
	}
	
	return 0;
}
