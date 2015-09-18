/***************************************************************************
 *   Copyright (C) 2012 by Robot Group Goettingen                          *
 *                                    									   *
 *    fhesse@physik3.gwdg.de     			                               *
 *    xiong@physik3.gwdg.de                  	                           *
 *    poramate@physik3.gwdg.de                                             *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                                                         *
 *   AMOSII v2 has now only 18 sensors                                     *
 ***************************************************************************/

#include <selforg/agent.h>
#include <selforg/abstractrobot.h>

#include <selforg/one2onewiring.h>
#include <selforg/sinecontroller.h>
//#include <ode_robots/amosiistdscalingwiring.h>

#include "controllers/amosi/modular_neural_control/amosIIcontrol.h"

#include "amosIserial.h"
#include <cmath>

#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // terminal control definitions
#include <time.h>   // time calls
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <curses.h>

//ROS Stuff
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include <sstream>




using namespace lpzrobots;

using namespace std;

bool stop = 0;
double noise = .0;
double realtimefactor = 1;
bool singleline = true; // whether to display the state in a single line
double cpg = 0.05;

int rosInput_old;
float rosInput;
int threshold = 0;
bool newInput = false;
float position;

float rosInputScaled =0;
float activity = 0;
float w_pfs_rfs = 1.0;
float w_pfs_pfs = 3.5;
float neuron_output = 0;

ros::Publisher chatter_pub;
ros::Publisher chatter_pub2;

std_msgs::Float32 message;
std_msgs::Float32 message2;



// Helper
int contains(char **list, int len, const char *str) {
	for (int i = 0; i < len; i++) {
		if (strcmp(list[i], str) == 0)
			return i + 1;
	}
	return 0;
}
;

//ROS Callback
void RosCallback(const std_msgs::Float32::ConstPtr& msg) {
	//ROS_INFO_STREAM(msg->data);
	//chatter_pub.publish(msg);
	rosInput=msg->data;
	newInput = true;



}



int main(int argc, char** argv) {


	//Init ROS
		ros::init(argc, argv, "amosI_real_listener");
		//Init the Node
		ros::NodeHandle n;
		//Init Subscriber
		ros::Subscriber sub = n.subscribe("extractor", 1000, RosCallback);
		chatter_pub = n.advertise<std_msgs::Float32>("amos_real_pub", 1000);
		chatter_pub2 = n.advertise<std_msgs::Float32>("amos_real_pub2", 1000);
		//End of ROS stuff



	list<PlotOption> plotoptions;
	int port = 1;

	int index = contains(argv, argc, "-g");
	if (index > 0 && argc > index) {
		plotoptions.push_back(PlotOption(GuiLogger, atoi(argv[index])));
	}
	if (contains(argv, argc, "-f") != 0)
		plotoptions.push_back(PlotOption(File));
	if (contains(argv, argc, "-n") != 0)
		plotoptions.push_back(PlotOption(MatrixViz));
	if (contains(argv, argc, "-l") != 0)
		singleline = false;
	index = contains(argv, argc, "-p");
	if (index > 0 && argc > index)
		port = atoi(argv[index]);
	if (contains(argv, argc, "-h") != 0) {
		printf("Usage: %s [-g N] [-f] [-n] [p PORT]\n", argv[0]);
		printf("\t-g N\tstart guilogger with interval N\n\t-f\twrite logfile\n");
		printf("\t-n\tstart neuronviz\n");
		printf("\t-p PORT\t COM port number, default 1\n");
		printf("\t-h\tdisplay this help\n");
		exit(0);
	}

	GlobalData globaldata;
	AmosIISerialV2* robot;
	Agent* agent;
	initializeConsole();

	// Different controllers

	AbstractController* controller = new AmosIIControl();
	((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->changeControlInput(cpg);

	// one2onewiring gives full range of robot actuators
	AbstractWiring* wiring = new One2OneWiring(new WhiteUniformNoise(), true);

	//****************Finetuning for real robot experiments
	//((AmosIIControl*) controller)->  ---Access to controller parameters
	//see  $(AMOSIICONT)/amosIIcontrol.cpp for controller classes
	for (unsigned int i = TR0_m; i < (BJ_m); i++) {
		((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->motormap.at(i)->max_ctr = 130;
		((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->motormap.at(i)->max_ctr_offset = 120;
	}
	((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25) = 1.5;
	((AmosIIControl*) controller)->preprocessing_learning.rho1.at(26) = 1.5;

	//robot         = new AmosIISerialV2("/dev/ttyS0");     // using serial port
	robot = new AmosIISerialV2("/dev/ttyUSB0"); // using USB-to-serial adapter


	agent = new Agent(plotoptions);
	agent->init(controller, robot, wiring);
	// if you like, you can keep track of the robot with the following line.
	// this assumes that your robot returns its position, speed and orientation.
	// agent->setTracMkOptions(TrackRobot(true,false,false, false,"systemtest"));

	globaldata.agents.push_back(agent);
	globaldata.configs.push_back(robot);
	globaldata.configs.push_back(controller);


	showParams(globaldata.configs);
	printf("\nPress c to invoke parameter input shell\n");
	printf("The output of the program is more fun then useful ;-).\n");
	printf(" The number are the sensors and the position there value.\n");
	printf(" You probably want to use the guilogger with e.g.: -g 10\n");

	cmd_handler_init();
	long int t=0;

	initscr();
	cbreak();
	//noecho();
	intrflush(stdscr,FALSE);
	keypad(stdscr,TRUE);
	nodelay(stdscr,TRUE);

	cout<<"Options: Press a= Obstacle Avoidance ON/OFF"<<endl;
	cout<<"Options: Press b= BJC ON/OFF"<<endl;
	cout<<"Options: Press e= Reflex ON/OFF"<<endl;

	while(!stop) {
		agent->step(noise,t);
		position = ((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->cpg_output.at(0);
		//position = 10;//robot->legpos;
		position = (position-500)*10;
		message.data = position-2000;
		chatter_pub.publish(message);

		if (newInput)
		{
			//Scale input from +-2048 to +-1
			//std::cout << rosInput << endl;
			rosInputScaled = rosInput;
			//rosInputScaled = rosInputScaled/2048;
			//std::cout << rosInputScaled << endl;
			//Low Pass filter with single recurrent neuron
			//activity = rosInputScaled * w_pfs_rfs + neuron_output * w_pfs_pfs;
			//neuron_output = tanh(activity);
			//std::cout << neuron_output << endl;
			//message.data = neuron_output;
			//message2.data = rosInputScaled;
			//chatter_pub.publish(message);
			//chatter_pub2.publish(message2);

			if (rosInputScaled == -3000)
			{
				std::cout << "CPG Changed" << endl;

				cpg = 0.08;
				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->changeControlInput(cpg);

				std::cout << cpg << endl;
			}
			else if (rosInputScaled == -3500)
			{
				std::cout << "CPG Changed" << endl;

				cpg = 0.18;
				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->changeControlInput(cpg);

				std::cout << cpg << endl;
			}

			else
			{
				std::cout << "CPG Changed" << endl;

				cpg = 0.02;
				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->changeControlInput(cpg);

				std::cout << cpg << endl;
			}


			/*//Check if there is an active threshold and change cpg
			if (rosInputScaled >=-2500)
			{
				std::cout << "CPG Changed" << endl;

				cpg = 0.25;
				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->changeControlInput(cpg);

				std::cout << cpg << endl;

			}
			else
			{
				cpg = 0.05;
				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->changeControlInput(cpg);
				std::cout << cpg << endl;
			}*/


		}


		int key=0;
		key = wgetch (stdscr);



		//KEYBOARD BJC OPTION
		if (key==98){ //B
			if (((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_backbonejoint) {
				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_backbonejoint = false;
				std::cout << "BJC is OFF" << endl;
			} else {
				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_backbonejoint = true;
				((AmosIIControl*) controller)->y.at(BJ_m) = 0.0;
				std::cout << "BJC is ON" << endl;
			}
		}

		if (key==97){ //A
			std::cout << "CPG Changed" << endl;
			if (((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->Control_input == 0.03)
			{
				cpg = 0.14;
				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->changeControlInput(cpg);
				std::cout << "0.14" << endl;
			}
			else
			{
				cpg = 0.03;
				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->changeControlInput(cpg);
				std::cout << "0.14" << endl;
			}



			/*if (((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_obstacle) {
				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_obstacle = false;
				std::cout << "OA is OFF" << endl;
			} else {
				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_obstacle = true;
				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_reflexes=false;
				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_irreflexes=false;
				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_purefootsignal=false;
				std::cout << "OA is ON" << endl;
			}*/
		}


		if (key==101){ //E
			if (((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_allreflexactions) {
				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_allreflexactions = false;
				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_reflexes=false;
				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_irreflexes=false;
				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_purefootsignal=false;
				std::cout << "Reflex is OFF" << endl;
			} else {
				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_allreflexactions = true;
				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_reflexes=true;
				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_irreflexes= true;
				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_purefootsignal=true;
				std::cout << "Reflex is ON" << endl;


			}
		}


		if(control_c_pressed()) {

			if(!handleConsole(globaldata)) {
				stop=1;
			}
			cmd_end_input();
		}


		t++;

		ros::spinOnce();
	};
	delete robot;
	delete agent;
	closeConsole();
	fprintf(stderr,"terminating\n");
	// should clean up but what costs the world
	return 0;
}
