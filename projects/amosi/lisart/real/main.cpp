/***************************************************************************
 *   Copyright (C) 2012 by Robot Group Goettingen                          *
 *                                    					   *
 *    fhesse@physik3.gwdg.de     			                   *
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
#include <ncurses.h>
#include <sstream>
#include <linux/i2c-dev.h>
#include <signal.h>
#include <pigpio.h>

#include "defs.h"
#include "utils/real_robots/sensors/lizard_ear/lizard_ear.h"



using namespace lpzrobots;

using namespace std;

bool stop = 0;
double noise = .0;
double realtimefactor = 1;
bool singleline = true; // whether to display the state in a single line
double cpg = 0.14;

float activity = 0;
float w_pfs_rfs = 1.0;
float w_pfs_pfs = 3.5;
float neuron_output = 0;
double lizard_val=0;



// Helper
int contains(char **list, int len, const char *str) {
	for (int i = 0; i < len; i++) {
		if (strcmp(list[i], str) == 0)
			return i + 1;
	}
	return 0;
}
;




int main(int argc, char** argv)
{
	int i2c_handle;

        unsigned char channel_no, run_filter = 0;
        char rxbuf[BUF_SIZE] = {0}, cfg;

        unsigned int raw_data_ch1[N_SAMPLES_PER_CHANNEL], raw_data_ch2[N_SAMPLES_PER_CHANNEL];
        //unsigned int raw_data_ch3[N_SAMPLES_PER_CHANNEL], raw_data_ch4[N_SAMPLES_PER_CHANNEL];
        unsigned int i, j, bytecnt;

        double sine_L[N_SAMPLES_USED], sine_R[N_SAMPLES_USED];

        lizard_ear ear;

        // Initialise pigpio library
        if (gpioInitialise() < 0)
        {
                cout << "GPIO init failed." << endl;
                exit(1);
        }

        // Open I2C device
        i2c_handle = i2cOpen(1, PMODAD2_I2C_ADDRESS, 0);
        if (i2c_handle < 0)
        {
                cout << "Cannot open I2C device." << endl;
                exit(1);
        }

        // Set ADC configuration byte to read all 4 analogue channels
        cfg = ALL_CHANNELS | AD7991_VREF_VCC | AD7991_FLT_ON | AD7991_BTD_ON | AD7991_SD_OFF;

	// Write the configuration byte into the ADC
        i2cWriteDevice(i2c_handle, &cfg, 1);

        // Sleep for a while...
        usleep(100000);

        cout << "ADC configured..." << endl;

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
	//Set Initial CPG
	((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->changeControlInput(cpg);

	//Turn off BJC
	((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_backbonejoint = false;
	std::cout << "BJC is OFF" << endl;

	//Turn off reflexes
	((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_allreflexactions = false;
	((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_reflexes=false;
	((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_irreflexes=false;
	((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_purefootsignal=false;
	std::cout << "Reflex is OFF" << endl;

	// one2onewiring gives full range of robot actuators
	AbstractWiring* wiring = new One2OneWiring(new WhiteUniformNoise(), true);

	//Finetuning for real robot experiments
	//((AmosIIControl*) controller)->  ---Access to controller parameters
	//see  $(AMOSIICONT)/amosIIcontrol.cpp for controller classes
	for (unsigned int i = TR0_m; i < (BJ_m); i++) {
		((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->motormap.at(i)->max_ctr = 130;
		((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->motormap.at(i)->max_ctr_offset = 120;
	}
	((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25) = 1.5;
	((AmosIIControl*) controller)->preprocessing_learning.rho1.at(26) = 1.5;

	//robot = new AmosIISerialV2("/dev/ttyAMA0");     // using serial port
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
	//raw();
	cbreak();
	//noecho();
	intrflush(stdscr,FALSE);
	keypad(stdscr,TRUE);
	nodelay(stdscr,TRUE);

//	mvprintw(0,0,"Options: Press a= Obstacle Avoidance ON/OFF");
//	mvprintw(1,0,"Options: Press b= BJC ON/OFF");
//	mvprintw(2,0,"Options: Press e= Reflex ON/OFF");

	//cout<<"Options: Press a= Obstacle Avoidance ON/OFF"<<endl;
	//cout<<"Options: Press b= BJC ON/OFF"<<endl;
	//cout<<"Options: Press e= Reflex ON/OFF"<<endl;

	unsigned long cnt = 0;
	bool go = 0;
	cout << "0" << endl;
	//mvprintw(0,0,"0 ");


	while(!stop)
	{
		//cout << "1" << endl;

		//int ch = getch();
		//if (ch == 3)
                //{
		//	delete robot;
		//	delete agent;
                //	endwin();
                //	i2cClose(i2c_handle);
                //	gpioTerminate();
                //	system("stty sane");
                //	exit(0);
                //}

		agent->step(noise,t);

		//position = ((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->cpg_output.at(0);
		//position = 10;//robot->legpos;

		//-----------------------------------------------------------------------------------//
		//---------------------------Lizard Auditory System Class Here-----------------------//
		//-----------------------------------------------------------------------------------//

                // Read data samples from all 4 channels
                bytecnt = i2cReadDevice(i2c_handle, rxbuf, BUF_SIZE);
		//cout << "2" << endl;

                if (bytecnt == BUF_SIZE)
                {
                        //mvprintw(0,0,"bytecnt = %d",bytecnt);
			//cout << "bytecnt=" << bytecnt << endl;
                        for (i = 0, j = 0; i < BUF_SIZE; i+=2)
                        {
                                channel_no = (rxbuf[i] & 0x30) >> 4;
                                if ((channel_no == 0))
                                {
                                        raw_data_ch1[j++] = (unsigned int)((rxbuf[i] & 0x0F)<<8) + (unsigned int)rxbuf[i+1];
                                }
                        }
                        for (i = 0, j = 0; i < BUF_SIZE; i+=2)
                        {
                                channel_no = (rxbuf[i] & 0x30) >> 4;
                                if ((channel_no == 1))
                                {
                                        raw_data_ch2[j++] = (unsigned int)((rxbuf[i] & 0x0F)<<8) + (unsigned int)rxbuf[i+1];
                                }
                        }
                        run_filter = 1;

                }
                else
                {
                        //mvprintw(0,0,"bytecnt = %d",bytecnt);
			//cout << "bytecnt=" << bytecnt << endl;
                        run_filter = 0;
                }

                if (run_filter)
                {
                        run_filter = 0;
                        for (i = 0, j = 0; i < N_SAMPLES_USED; i++, j++)
                        {
                                sine_L[j] = (double)(raw_data_ch1[i]-0)/1;
                                sine_R[j] = (double)(raw_data_ch2[i]-0)/1;
                        }

                        ear.filter(sine_L, sine_R);

			//double threshold_low=-1;
			//double threshold_high=0.3;

			if (cnt++ != 50)
			{
				lizard_val += 20*(log10(ear.sumL)-log10(ear.sumR));
			}
			else
			{
				lizard_val /= 50;
				signed int temp = (signed int)(lizard_val*100);
				cout << temp << endl;
				//mvprintw(1,0,"lizard_val=%f",lizard_val);
				cnt = 0;

				if (temp < (signed int)-100)
				{
					cout << "Left" << endl;
					//Turn Left
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->setInputNeuronInput(0, 0);
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->setInputNeuronInput(1, 1);
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->setInputNeuronInput(2, 1);
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->setInputNeuronInput(3, 1);
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->setInputNeuronInput(4, -1);
				}
				else if (temp > (signed int)30)
				{
					cout << "Right" << endl;

					//Turn Right
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->setInputNeuronInput(0, 0);
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->setInputNeuronInput(1, 1);
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->setInputNeuronInput(2, 1);
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->setInputNeuronInput(3, -1);
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->setInputNeuronInput(4, 1);
				}
				else
				{
					cout << "Fwd." << endl;
					//Go Forward
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->setInputNeuronInput(0, 0);
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->setInputNeuronInput(1, 1);
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->setInputNeuronInput(2, 1);
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->setInputNeuronInput(3, -1);
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->nlc->setInputNeuronInput(4, -1);
				}
				cout << "End" << endl;
				lizard_val = 0;
			}
		}

		//int key=0;
		//key = wgetch (stdscr);

/*
		//Keyboard lizard_val
		if (key==98)//B
		{
			lizard_val-=0.2;
			/std::cout << "lizard_val:" << lizard_val << endl;
			//std::cout << "0.14" << endl;
		}

		if (key==101)//E
		{
			lizard_val+=0.2;
			std::cout << "lizard_val:" << lizard_val << endl;
		}

		if (key==97) //A
		{
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

			//if (((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_obstacle)
			//{
			//	((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_obstacle = false;
			//	std::cout << "OA is OFF" << endl;
			//}
			//else
			//{
			//	((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_obstacle = true;
			//	((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_reflexes=false;
			//	((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_irreflexes=false;
			//	((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_purefootsignal=false;
			//	std::cout << "OA is ON" << endl;
			//}

		}
*/
/*
		if(control_c_pressed())
		{

			if(!handleConsole(globaldata))
			{
				stop=1;
			}
			cmd_end_input();
		}
*/
		t++;
	};

	delete robot;
	delete agent;
	closeConsole();
	//fprintf(stderr,"terminating\n");
	return 0;
}
