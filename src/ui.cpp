#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "second_assignment/KeyboardInput.h"
#include "std_srvs/Empty.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

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

char command;
ros::ServiceClient client; 
ros::ServiceClient client_restart; 
second_assignment::KeyboardInput my_input;
std_srvs::Empty restart_srv;


int main(int argc, char **argv)
{
	cout<< "\n" BHYEL "################# USER INTERFACE #################" RESET "\n";
	cout<< BHGRN "Press 'a' for increasing the robot velocity!" RESET"\n"; 
	cout<< BHBLU "Press 's' for decreasing the robot velocity!" RESET"\n"; 
	cout<< BHCYN "Press 'r' for resetting the robot initial position and velocity!" RESET"\n"; 
	cout<< BHRED "Press 'q' for quitting the applicaiton!" RESET"\n";
	
	ros::init(argc, argv, "ui");
	ros::NodeHandle n;
	
	client =  n.serviceClient<second_assignment::KeyboardInput>("/keyboard_input");
	client_restart =  n.serviceClient<std_srvs::Empty>("/reset_positions");
	
	

	while(command!='q') {
		cin>>command;
		
		if(command=='a'){
			my_input.request.input='a';
			client.waitForExistence();
			client.call(my_input);
			cout<< BHGRN "Increasing..." RESET "\n";
		}
		else if(command=='s'){
			my_input.request.input='s';
			client.waitForExistence();
			client.call(my_input);
			cout<< BHBLU "Decreasing..." RESET "\n";
		}
		else if(command=='r'){
			client_restart.waitForExistence();
			client_restart.call(restart_srv);
			my_input.request.input='s';
			client.waitForExistence();
			client.call(my_input);
			cout<< BHCYN "Restarting..." RESET "\n";
		}
		else if(command=='q'){
			my_input.request.input='q';
			client.call(my_input);
			cout<< BHRED "Exiting..." RESET "\n";
			return 0;
			
		}
		else{
			cout<< BHRED "This command is not allowed, please use the commands above!" RESET "\n";
		}
	}
	
	return 0;
}
