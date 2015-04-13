/***************************************************************************
 *   Copyright (C) 2005-2013 LpzRobots development team                    *
 *   Authors:                                                              *
 *    Simón Smith <artificialsimon at ed dot ac dot uk>                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 ***************************************************************************/

#include "basiccontroller.h"

#include <assert.h>
using namespace std;
using namespace matrix;

BasicController::BasicController(const std::string& name)
: AbstractController(name, "1.0") {
	initialised=false;
	// add threshold parameter to configurable parameters, setable on console
	addParameterDef("threshold", &threshold, 0.2, 0, 1, "threshold for IR-sensor");
	this->stepCounter = 0;
}

BasicController::BasicController(const std::string& name, std::string fileName, int sampleRate)
: BasicController(name) {
	//Init logging stuff
	this->sampleRate = sampleRate;
	this->skipSteps = 3 * sampleRate;
	this->out.open(fileName.c_str());
	outAcc.open("SLAM/sensorData/accData.dat");
	out << "#SampleRate: "<<this->sampleRate<<endl;
	out <<"#Step\tposX\tposY\tYaw\tvX\tvY\tvYaw\taX\taY\tRF1\tRF2\tRF3\tRF4\tRF5\tRF6\tRF7..."<<endl;
	outAcc <<"#Step\taccX\taccY\tposX\tposY\tYaw\tvX\tvY\tvYaw"<<endl;
}



void BasicController::stepNoLearning(const sensor* sensors, int number_sensors,
		motor* motors, int number_motors) {
	// Very simple controller, if IR sensors are higher than threshold then
	// the robot will turn oposite direction, or backward when to close
	// Sensors indices and motor indices can be accessed by the SIdx and MIdx functions
	// provided by the controller base class
	if (sensors[SIdx("IR front left")] > 2*threshold ||
			sensors[SIdx("IR front right")] > 2*threshold) { // move backward
		motors[MIdx("left motor")] = -1.;
		motors[MIdx("right motor")] = -1.;
	}else if (sensors[SIdx("IR left")] > threshold ||
			sensors[SIdx("IR left front")] > threshold ||
			sensors[SIdx("IR front left")] > threshold) { // turn right
		motors[MIdx("left motor")] = .1;
		motors[MIdx("right motor")] = 1.;
	}
	else if (sensors[SIdx("IR right")] > threshold ||
			sensors[SIdx("IR right front")] > threshold ||
			sensors[SIdx("IR front right")] > threshold) {
		motors[MIdx("left motor")] = 1.;
		motors[MIdx("right motor")] = .1;
	}
	else { // Move forward
		motors[MIdx("left motor")] = 1.;
		motors[MIdx("right motor")] = 1.;
	}

	//Logging sensor data
	if(this->outAcc.is_open() && this->stepCounter >= this->skipSteps){
	  outAcc << this->stepCounter << "\t";
	  outAcc << sensors[SIdx("Linear Acceleration in x (body frame)")] << "\t" <<
	      sensors[SIdx("Linear Acceleration in y (body frame)")] << "\t" ;
	  outAcc<< -sensors[SIdx("Position X")] << "\t" <<
	      -sensors[SIdx("Position Y")]<<"\t"<<sensors[0]<<"\t";
	  outAcc << sensors[SIdx("Body Speed X")] << "\t" << sensors[SIdx("Body Speed Y")] << "\t" <<
	      sensors[SIdx("Angular velocity yaw")]<<endl;;
	}
	if(this->out.is_open() && this->stepCounter % this->sampleRate == 0 && this->stepCounter >= this->skipSteps){
		//NOTE: Somehow positions is measured with negative sign (at least with respect to
		//coordinate system used in my thesis
		out << this->stepCounter << "\t"<< -sensors[SIdx("Position X")] << "\t" <<
				-sensors[SIdx("Position Y")]<<"\t"<<sensors[0]<<"\t";
		out << sensors[SIdx("Body Speed X")] << "\t" << sensors[SIdx("Body Speed Y")] << "\t" <<
				sensors[SIdx("Angular velocity yaw")]<<"\t";
		out << sensors[SIdx("Linear Acceleration in x (body frame)")] << "\t" <<
				sensors[SIdx("Linear Acceleration in y (body frame)")] << "\t" ;
		//Range finder beams
		for(int i = 3; i <= 17; i++)
			out << sensors[i]<<"\t";
		out << endl;
	}

	this->stepCounter++;

}

void BasicController::step(const sensor* sensors, int sensornumber,
		motor* motors, int motornumber) {
	stepNoLearning(sensors,sensornumber, motors, motornumber);
}


void BasicController::init(int sensornumber, int motornumber, RandGen* randGen) {
	assert(motornumber >=2 && sensornumber >=8);
	// Set the number of sensors and motors
	nSensors = sensornumber;
	nMotors  = motornumber;
	initialised=true;
}


int BasicController::getSensorNumber() const {
	return nSensors;
}

int BasicController::getMotorNumber() const {
	return nMotors;
}

bool BasicController::store(FILE* f) const {
	//  S.store(f); // if S is a matrix::Matrix
	Configurable::print(f,0);
	return true;
}

bool BasicController::restore(FILE* f) {
	//  S.restore(f); // if S is a matrix::Matrix
	Configurable::parse(f);
	return true;
}
