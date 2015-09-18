/***************************************************************************
 *   Copyright (C) 2012 by Robot Group Goettingen                          *
 *                                    									                   *
 *    fhesse@physik3.gwdg.de     			                                     *
 *    xiong@physik3.gwdg.de                  	                             *
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

#include "amosIIcontrol.h"

#include "amosIIserialv1.h"  //serial interface to AMOSII version 1
#include "amosIIserialv2.h"  //serial interface to AMOSII version 2

#include <cmath>

#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // terminal control definitions
#include <time.h>   // time calls

#include <vector>

#include <curses.h>
#include <comedilib.h>
#include "ximuAccess.h"
#include "laserScanner.h"


using namespace lpzrobots;


using namespace std;

bool stop=0;
double noise=.0;
double realtimefactor=1;
bool   singleline = true; // whether to display the state in a single line

int sampleRateLRF = 50;
int sampleRateIMU = 1;




// Helper
int contains(char **list, int len,  const char *str){
	for(int i=0; i<len; i++){
		if(strcmp(list[i],str) == 0) return i+1;
	}
	return 0;
};

int main(int argc, char** argv){
	list<PlotOption> plotoptions;

	int index = contains(argv,argc,"-g");
	if(index >0 && argc>index) {
		plotoptions.push_back(PlotOption(GuiLogger, atoi(argv[index])));
	}
	if(contains(argv,argc,"-f")!=0) plotoptions.push_back(PlotOption(File));
	if(contains(argv,argc,"-n")!=0) plotoptions.push_back(PlotOption(MatrixViz));
	if(contains(argv,argc,"-l")!=0) singleline=false;
	index = contains(argv,argc,"-p");
	if(contains(argv,argc,"-h")!=0) {
		printf("Usage: %s [-g N] [-f] [-n] [p PORT]\n",argv[0]);
		printf("\t-g N\tstart guilogger with interval N\n\t-f\twrite logfile\n");
		printf("\t-n\tstart neuronviz\n");
		printf("\t-p PORT\t COM port number, default 1\n");
		printf("\t-h\tdisplay this help\n");
		exit(0);
	}

	GlobalData globaldata;
	AmosIISerialV1* robot;
	Agent* agent;
	initializeConsole();

	// Different controllers
	int amosVersion = 1;
	bool mMCPGs=false; //Single CPG-based (false) or multiple CPG-based(true) control
	bool useMuscleModel = false;
	AbstractController* controller = new AmosIIControl(amosVersion, mMCPGs, useMuscleModel);
	// one2onewiring gives full range of robot actuators
	AbstractWiring* wiring = new One2OneWiring(new WhiteUniformNoise(),true);

	 //robot         = new AmosIISerialV1("/dev/ttyS0");     // using serial port
	 robot         = new AmosIISerialV1("/dev/ttyUSB0");   // using USB-to-serial adapter

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

	cmd_handler_init();
	long int t=0;

	((AmosIIControl*) controller)->control_adaptiveclimbing[0]->switchon_obstacle = false;
	((AmosIIControl*) controller)->control_adaptiveclimbing[0]->switchon_reflexes=false;
	((AmosIIControl*) controller)->control_adaptiveclimbing[0]->switchon_irreflexes=false;
	((AmosIIControl*) controller)->control_adaptiveclimbing[0]->switchon_purefootsignal=false;

	//Connect LRF and IMU
	//xIMU
	XIMU::Ximu imuDevice("parameters.cfg");
	//LRF
	URG::URGLaser urg;
	if(!(imuDevice.isConnected() && urg.connect())){
		cout << imuDevice.isConnected() << " "<< urg.connected<<endl;
		cout << "Sensors not connected"<<endl;
		return 0;
	}

	//Init ncurses
	initscr();
	cbreak();
	//noecho();
	intrflush(stdscr,FALSE);
	keypad(stdscr,TRUE);
	nodelay(stdscr,TRUE);
	int key = 0;

	vector<vector<double> > positions, velocities, accelerations;
	vector<bool> turning;
	vector<int> indicesVelo, indicesLRF, indicesAcc;
	vector<vector<long> > urgData;
	bool isTurning = false;
//	imuDevice.startReading();
	bool logging = false;
	clock_t t_begin;

	while(!stop){
		agent->step(noise,t);
		key = wgetch (stdscr);

		switch(key){
		//Start logging on s
		case 115:
			move(3,0);
			clrtoeol();
			printw("Start logging");
			imuDevice.startReading();
			urg.updateData();
			turning.push_back(false);
			urgData.push_back(urg.data);
			indicesLRF.push_back(t);
			logging = true;
			t_begin = clock();
			break;
		//Quit on q
		case 113:
			move(3,0);
			clrtoeol();
			printw("Quitting");
			stop = true;
			break;
		//Movement with arrow keys
		case KEY_UP:
			move(3,0);
			clrtoeol();
			printw("Forward");
			((AmosIIControl*) controller)->control_adaptiveclimbing[0]->moveForward();
			isTurning = false;
			break;
		case KEY_DOWN:
			move(3,0);
			clrtoeol();
			printw("Backward");
			((AmosIIControl*) controller)->control_adaptiveclimbing[0]->moveBackward();
			isTurning = false;
			break;
		case KEY_LEFT:
			move(3,0);
			clrtoeol();
			printw("Left");
			((AmosIIControl*) controller)->control_adaptiveclimbing[0]->turnLeft();
			isTurning = true;
			break;
		case KEY_RIGHT:
			move(3,0);
			clrtoeol();
			printw("Right");
			((AmosIIControl*) controller)->control_adaptiveclimbing[0]->turnRight();
			isTurning = true;
			break;
		case 263: //Backspace
			move(3,0);
			clrtoeol();
			printw("Stop");
			((AmosIIControl*) controller)->control_adaptiveclimbing[0]->stop();
			isTurning = false;
			break;
		//Do nothing when no key is pressed
		case -1:
			break;
		//Print key number for unkown key
		default:
			move(3,0);
			clrtoeol();
			printw("Key: %i", key);
		}

		mvprintw(2,0, "Step: %i", t);

//		if(t % sampleRateIMU == 0 && logging){
//			//If robot is turning, forward velocity is zero
////			if(isTurning)
//			positions.push_back(imuDevice.getPosition());
//			velocities.push_back(imuDevice.getVelocity());
//			accelerations.push_back(imuDevice.getAcceleration());
//			indicesVelo.push_back(t);
////			imuDevice.resetVelocity();
//		}

		if(t % sampleRateLRF == 0 && logging){
			urg.updateData();
			urgData.push_back(urg.data);
//			turning.push_back(isTurning);
			indicesLRF.push_back(t);
			indicesAcc.push_back(imuDevice.accData.size()-1);
//			imuDevice.resetVelocity();
		}


		t++;
	};
	imuDevice.stopReading();
	imuDevice.disconnect();
	urg.disconnect();
	clock_t t_end = clock();
	double elapsed_secs = double(t_end - t_begin) / CLOCKS_PER_SEC;
	endwin();
	closeConsole();
	cout<<"Time step: "<<elapsed_secs/ (double)positions.size()<<endl;

	//TODO: Write data to files
	imuDevice.writeDataToFile("data/dataIMU.dat");
	string fileName = "data/velocities.dat";
	ofstream out(fileName.c_str());
	out << "#Time step [sec]: "<<elapsed_secs/ (double)positions.size()<<endl;
	out<<"#step\tposX\tposY\tposZ\tvX\tvY\tvZ\tvYaw\taccX\taccY"<<std::endl;
	for(int i = 0; i < (int)positions.size(); i++){
		out<<indicesVelo[i]<<"\t"<<positions[i][0]<<"\t"<<positions[i][1]<<"\t"<<positions[i][2]<<"\t";
		out<<velocities[i][0]<<"\t"<<velocities[i][1]<<"\t"<<velocities[i][2]<<"\t"<<velocities[i][5]<<
				"\t"<<accelerations[i][0]<<"\t"<<accelerations[i][1]<<std::endl;
	}
	out.close();
	///Write LRF
	std::string baseFileName = "data/lrf";
	std::stringstream ss;
	for(int i = 0; i < (int)urgData.size(); i++){
		ss << indicesLRF[i];
		fileName  = baseFileName + ss.str() + ".dat";
		out.open(fileName.c_str());
		out << '!'<<'\t'<<indicesAcc[i]<<endl;
//		out << '!'<<'\t'<<turning[i]<<endl;
		out << "#Step\tLength"<<endl;
		for(int j = 0; j < (int)urgData[i].size(); j++)
			out << j << "\t"<< urgData[i][j]<<endl;
		ss.str("");
		ss.clear();
		out.close();
	}



using namespace lpzrobots;


using namespace std;

bool stop=0;
double noise=.0;
double realtimefactor=1;
bool   singleline = true; // whether to display the state in a single line


// Helper
int contains(char **list, int len,  const char *str){
	for(int i=0; i<len; i++){
		if(strcmp(list[i],str) == 0) return i+1;
	}
	return 0;
};

int main(int argc, char** argv){
	list<PlotOption> plotoptions;
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
	AmosIISerialV1* robot;
	Agent* agent;
	initializeConsole();

	// Different controllers
	int amosVersion = 1;
	bool mMCPGs=false; //Single CPG-based (false) or multiple CPG-based(true) control
	bool useMuscleModel = false;
	AbstractController* controller = new AmosIIControl(amosVersion, mMCPs, useMuscleModel);
	// one2onewiring gives full range of robot actuators
	AbstractWiring* wiring = new One2OneWiring(new WhiteUniformNoise(),true);

	 //robot         = new AmosIISerialV1("/dev/ttyS0");     // using serial port
	 robot         = new AmosIISerialV1("/dev/ttyUSB0");   // using USB-to-serial adapter

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

	cmd_handler_init();
	long int t=0;

	((AmosIIControl*) controller)->control_adaptiveclimbing[0]->switchon_obstacle = true;
	((AmosIIControl*) controller)->control_adaptiveclimbing[0]->switchon_reflexes=false;
	((AmosIIControl*) controller)->control_adaptiveclimbing[0]->switchon_irreflexes=false;
	((AmosIIControl*) controller)->control_adaptiveclimbing[0]->switchon_purefootsignal=false;

	while(!stop){
		agent->step(noise,t);

		if(control_c_pressed()){

			if(!handleConsole(globaldata)){
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
