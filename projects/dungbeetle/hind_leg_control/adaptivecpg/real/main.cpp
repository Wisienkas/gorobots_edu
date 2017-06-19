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

#include "controllers/dungbeetle/hind_leg_control/adaptivecpg/hindLegControl.h"

#include <utils/real_robots/dungbeetle/dungBeetleSerialV2.h>  //serial interface to AMOSII version 1
//serial interface to AMOSII version 2
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
using namespace lpzrobots;

using namespace std;

bool stop = 0;
double noise = .0;
double realtimefactor = 1;
bool singleline = true; // whether to display the state in a single line


// Helper
int contains(char **list, int len, const char *str) {
	for (int i = 0; i < len; i++) {
		if (strcmp(list[i], str) == 0)
			return i + 1;
	}
	return 0;
}
;

int main(int argc, char** argv) {
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
	dungBeetleSerial* robot;
	Agent* agent;
	initializeConsole();

	// Different controllers

	AbstractController* controller = new hindLegControl();
	// one2onewiring gives full range of robot actuators
	AbstractWiring* wiring = new One2OneWiring(new WhiteUniformNoise(), true);

	//****************Finetuning for real robot experiments
	//((AmosIIControl*) controller)->  ---Access to controller parameters
	//see  $(AMOSIICONT)/amosIIcontrol.cpp for controller classes

	//robot         = new AmosIISerialV2("/dev/ttyS0");     // using serial port
	robot = new dungBeetleSerial("/dev/ttyUSB0"); // using USB-to-serial adapter


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

	/*cout<<"Options: Press a= Obstacle Avoidance ON/OFF"<<endl;
	cout<<"Options: Press b= BJC ON/OFF"<<endl;
	cout<<"Options: Press e= Reflex ON/OFF"<<endl;

	cout << "hi brf"<< endl;
	*/while(!stop) {//!stop

		agent->step(noise,t);
		//std::cout << "hifgvf" << t <<endl;


		int key=0;
		key = wgetch (stdscr);
		if(key==98)
		{
			//((hindLegControl*) controller)->y.at(1) = -1;
			for(int i=0;i<DUNGBEETLE_MOTOR_MAX;i++)
			{


				((hindLegControl*) controller)->y.at(i) = -1;
				std::cout << "BJC is OFF" << endl;
				//y_[i]=y.at(i);//SET CPG VALUE HERE
			}

		}
		/*
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

		 */if (key==97){ //A
			 //((hindLegControl*) controller)->y.at(1) = -1;
			 for(int i=0;i<DUNGBEETLE_MOTOR_MAX;i++)
			 {


				 ((hindLegControl*) controller)->y.at(i) = +1;
				 std::cout << "BJC is OFF" << endl;
				 //y_[i]=y.at(i);//SET CPG VALUE HERE
			 }

		 }

		 /*
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
		  */

		 if(control_c_pressed()) {

			 if(!handleConsole(globaldata)) {
				 stop=1;
			 }
			 cmd_end_input();
		 }


		 t++;
	};
	delete robot;
	delete agent;
	closeConsole();
	fprintf(stderr,"terminating\n");
	// should clean up but what costs the world
	return 0;
}
