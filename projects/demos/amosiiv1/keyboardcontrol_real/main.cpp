
	#include <sys/mman.h>
	#include <unistd.h>
	#include <sys/types.h>
	#include <sys/stat.h>
	#include <fcntl.h>
	#include <stdio.h>
	#include <sys/ioctl.h>
	#include <stdlib.h>
	#include <curses.h>



/*
 *  Template/Example for main program communicating with real amosII using serial communication
 * copy to own folder (including make file)
 * add your controller files to the folder
 * include your controller in this main file and in Makefile.conf and compile
 *  -> you should be able to test your controller now on the real machine
 *
 * */




#include <selforg/agent.h>
#include <selforg/abstractrobot.h>

#include <selforg/one2onewiring.h>
#include <selforg/sinecontroller.h>
//#include <ode_robots/amosiistdscalingwiring.h>



// instead of tripodgait18dof.h you can add your controller here
//#include "tripodgait18dof.h"
#include "amosIIcontrol.h"

#include "amosIIserial.h"


#include <cmath>

#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // terminal control definitions
#include <time.h>   // time calls


using namespace lpzrobots;


using namespace std;

bool stop=0;
double noise=.0;
double realtimefactor=1;
bool   singleline = true; // whether to display the state in a single line



// Helper
int contains(char **list, int len,  const char *str){	for(int i=0; i<len; i++){
		if(strcmp(list[i],str) == 0) return i+1;
	}
	return 0;
};

int main(int argc, char** argv){	list<PlotOption> plotoptions;
	int port = 1;

	int index = contains(argv,argc,"-g");
	if(index >0 && argc>index) {
		plotoptions.push_back(PlotOption(GuiLogger, atoi(argv[index])));
	}
	if(contains(argv,argc,"-f")!=0) plotoptions.push_back(PlotOption(File));
	if(contains(argv,argc,"-n")!=0) plotoptions.push_back(PlotOption(MatrixViz));
	if(contains(argv,argc,"-l")!=0) singleline=false;
	index = contains(argv,argc,"-p");
	if(index >0 && argc>index)
		port=atoi(argv[index]);
	if(contains(argv,argc,"-h")!=0) {
		printf("Usage: %s [-g N] [-f] [-n] [p PORT]\n",argv[0]);
		printf("\t-g N\tstart guilogger with interval N\n\t-f\twrite logfile\n");
		printf("\t-n\tstart neuronviz\n");
		printf("\t-p PORT\t COM port number, default 1\n");
		printf("\t-h\tdisplay this help\n");
		exit(0);
	}

	GlobalData globaldata;
	AmosIISerial* robot;
	Agent* agent;
	initializeConsole();

	// Different controllers


	//	// Standard Tripod controller
	//	AbstractController* controller = new TripodGait18DOF();
	//	// create wiring between robot and controller (sensor <--> motor)
	//	// the TripodGait18DOF assumes a the normal walking range  (a limited one)
	//	// and hence requires the AmosIIStdScalingWiring:
	//	AbstractWiring* wiring = new AmosIIStdScalingWiring(new ColorUniformNoise(0.0));


	//AbstractController*
	AmosIIControl* controller = new AmosIIControl();
	// one2onewiring gives full range of robot actuators
	AbstractWiring* wiring = new One2OneWiring(new WhiteUniformNoise(),true);

	// using serial port
	robot         = new AmosIISerial("/dev/ttyS0");
    // using USB-to-serial adapter
//	robot         = new AmosIISerial("/dev/ttyUSB0");

	agent         = new Agent(plotoptions);
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

	bool use_additional_keys = true;
	if (!use_additional_keys){
		cmd_handler_init();
		long int t=0;
		while(!stop){
			agent->step(noise,t);

			//		printf("after agent step\n");
			//		std::cout<<"after agent step"<<std::endl;
			if(control_c_pressed()){
				//			printf("after pressing STRL-C\n");
				//			std::cout<<"after agent step"<<std::endl;
				if(!handleConsole(globaldata)){
					stop=1;
				}
				cmd_end_input();
			}
			// int drawinterval = 10000;
			// if(realtimefactor!=0){
			//   drawinterval = max(1,int(6*realtimefactor));
			// }
			// if(t%drawinterval==0){
			//   printRobot(robot);
			//   usleep(60000);
			// }
			t++;
		};
	} else { // use additional keys, setting of parameters by pressing Control_C not possible
		std::cout<<"started with walking = false"<<std::endl;
		initscr();
		cbreak();
		//noecho();
		intrflush(stdscr,FALSE);
		keypad(stdscr,TRUE);
		nodelay(stdscr,TRUE);
		long int t=0;
		bool walking = false;
		// add in order to add a keystroke for stopping coming back
		bool come_back = false;
		while(!stop){

			int key=0;

			key = wgetch (stdscr);
			if (key==115){
					std::cout<<"S pressed"<<std::endl;
					if (walking){
						controller->control_adaptiveclimbing.tripod.stopWalking();
						std::cout<<"stop now"<< std::endl;
						walking=false;
					} else{
						controller->control_adaptiveclimbing.tripod.startWalking();
						std::cout<<"start now"<< std::endl;
						walking=true;
					}
				}


				if (key==104){
					controller->control_adaptiveclimbing.tripod.liftLeg();
					std::cout<<"H pressed"<<std::endl;
					std::cout<<"lift leg"<<std::endl;
				}

				if (key==106){
					controller->control_adaptiveclimbing.tripod.lowerLeg();
					std::cout<<"J pressed"<<std::endl;
					std::cout<<"lower leg"<<std::endl;
				}

				if (key==98){
					std::cout<<"B pressed"<<std::endl;
//					if (come_back){
//						controller->control_adaptiveclimbing.tripod.stopComeBack();
//						std::cout<<"stop come back"<<std::endl;
//						come_back=false;
//					} else {
						controller->control_adaptiveclimbing.tripod.comeBack();
						std::cout<<"lower leg and come back"<<std::endl;
		//				come_back=true;
	//				}
				}


				if (key==27){
					std::cout<<"ESC pressed"<<std::endl;
					stop=true;
				}

				if (key==108){ // Turn left
					std::cout<<"L pressed"<<std::endl;
						controller->control_adaptiveclimbing.tripod.turnLeft();
						std::cout<<"start now"<< std::endl;
				}


				if (key==114){ // Turn right
					std::cout<<"R pressed"<<std::endl;
						controller->control_adaptiveclimbing.tripod.turnRight();
						std::cout<<"start now"<< std::endl;
				}

				if (key==97){ // left side ward
					std::cout<<"A pressed"<<std::endl;
						controller->control_adaptiveclimbing.tripod.sidewardleft();
						std::cout<<"start now"<< std::endl;
				}


				if (key==103){ // right side ward
					std::cout<<"G pressed"<<std::endl;
						controller->control_adaptiveclimbing.tripod.sidewardright();
						std::cout<<"start now"<< std::endl;
				}

				if (key==121){ // Turn right side ward // STILL NOT WORKING this function
						std::cout<<"Y pressed"<<std::endl;
							controller->control_adaptiveclimbing.tripod.turnrightsideward();
							std::cout<<"start now"<< std::endl;
				}

				if (key==119){ // Turn right side ward
						std::cout<<"W pressed"<<std::endl;
							controller->control_adaptiveclimbing.tripod.turnleftsideward();
							std::cout<<"start now"<< std::endl;
				}

			agent->step(noise,t); // execute a step in controller and robot
			t++;
		}
	}





	delete robot;
	delete agent;
	closeConsole();
	fprintf(stderr,"terminating\n");
	// should clean up but what costs the world
	return 0;
}



