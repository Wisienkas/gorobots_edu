#include "rugWarrior.h"
#include <curses.h>
#include "ximuAccess.h"

#include <iostream>
#include <fstream>
#include <numeric>
#include <unistd.h>

#include <boost/chrono.hpp>

using namespace std;
using namespace XIMU;
using namespace RWControl;
using namespace boost::chrono;

//sudo rmmod ftdi_sio
//sudo rmmod usbserial

int main(){
//	Ximu imuDevice("parameters.cfg");
//	imuDevice.startReading();
//	high_resolution_clock::time_point t1 = high_resolution_clock::now();
//	while(imuDevice.accData.size() < 5e4){
//		cout << imuDevice.accData.size()<<endl;
//		usleep(1000000);
//	}
//	high_resolution_clock::time_point t2 = high_resolution_clock::now();
//	duration<double> totalTime = t2 - t1;
//	double measureRate = 1 / (totalTime.count()/imuDevice.accData.size());
//	cout << "Measure rate: "<<measureRate<<" hz"<<endl;
//	cout << "Sample time: "<<1./measureRate<<" sec"<<endl;
//	return 1;


	RugWarrior robot("parameters.cfg");
	if(!robot.isConnected()){
		cout << "Device not connected"<<endl;
		return -1;
	}
//	usleep(5000000);
	//robot.moveForward();
//	robot.startObstacleAvoid();
	initscr();
	cbreak();
	notimeout(stdscr, true);
	keypad(stdscr, true);
	int key=0;
	/* looped grabbing, differencing and displaying */
	while (key != 27){
		key = wgetch(stdscr);
		mvprintw(1,0,"%3d",key);
		switch(key){
		//Start logging on s key
		case 115:
			mvprintw(2,0, "START LOGGING");
			robot.startLogging();
			break;
		case KEY_UP:
			mvprintw(2,0, "UP");
			robot.moveForward();
			break;
		case KEY_DOWN:
			mvprintw(2,0, "DOWN");
			robot.moveBackward();
			break;
		case KEY_LEFT:
			mvprintw(2,0, "LEFT");
			robot.turnLeft();
//			robot.setMotors(3900, 3000);
			break;
		case KEY_RIGHT:
			mvprintw(2,0, "RIGHT");
			robot.turnRight();
//			robot.setMotors(3000, 3900);
			break;
		case 263: //Backspace
			mvprintw(2,0, "STOP");
			robot.stop();
			break;
		default:
			mvprintw(2,0, "NONE");
		}
	}
	robot.stopLogging();
	robot.stop();
	endwin();
	cout << "Mean dt: "<<robot.getSampleTime()<<" sec"<<endl;
	robot.writeDataToFile("SLAM/sensorData/");
//	robot.writeDataToFile("data/");
}
