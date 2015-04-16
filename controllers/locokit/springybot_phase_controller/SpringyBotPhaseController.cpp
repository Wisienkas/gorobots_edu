/*
 * LocoKitController.cpp
 *
 *  Created on: Dec 27, 2014
 *      Author: leon
 */

#include <assert.h>

#include "SpringyBotPhaseController.h"

using namespace std;
using namespace matrix;


SpringyBotPhaseController::SpringyBotPhaseController(const std::string& name, int trial)
: AbstractController(name, "1.0") {
	initialised=false;
	ready = false;
	ticks_since_init = ticks_since_reset = 0;
	addParameterDef("threshold", &threshold, 0.2, 0, 1, "threshold for IR-sensor");
	distanceMax = 0;
	time_per_tick = 0.01;
	trialNumber = trial+1;
	rearPositionMax = -10;
	tiltMax = -10;
	tiltMin = 10;


//	static double speedList[8] = {2.75, 3.0, 3.25, 3.5, 3.75, 4.0, 4.25, 4.5 };
//	static double phaseList[8] = {M_PI-0.3, M_PI-0.2, M_PI-0.1, M_PI, M_PI+0.1, M_PI+0.2, M_PI+0.3, M_PI+0.4};
//	int speedIndex = trial/8;
//	int phaseIndex = trial%8;
//	speedSetpoint = speedList[speedIndex];
//	phaseSetpoint = phaseList[phaseIndex];

//	int speed = trial/20;
//	int phase = trial%20;
//	speedSetpoint = 3.0 + (speed*0.1);
//	phaseSetpoint = M_PI + (phase*0.1)-1.0;

//	speedSetpoint = 2.0+(trial*0.1);
//	phaseSetpoint = M_PI-0.2;

	speedSetpoint = 8.0;
	phaseSetpoint = 4.0;
	std::cout << "Running trial:" << trialNumber << " with speed:" << speedSetpoint << " and phase:" << phaseSetpoint << std::endl;
}

SpringyBotPhaseController::~SpringyBotPhaseController() {
	//outputFile << std::endl;
	outputFile.close();
}

void SpringyBotPhaseController::stepNoLearning(const sensor* sensors, int number_sensors, motor* motors, int number_motors) {
}

void SpringyBotPhaseController::step(const sensor* sensors, int sensornumber, motor* motors, int motornumber) {

	// Update internal time
	ticks_since_init++;
	ticks_since_reset++;

	// Read sensors
	double tilt = 						sensors[SIdx("SpeedTranslationalTranslationalRelRotationalRotationalRelY")];
	double leftFrontPosition = 			sensors[SIdx("left front motor")];
	double leftRearPosition = 			sensors[SIdx("left rear motor")];
	double rightFrontPosition = 		sensors[SIdx("right front motor")];
	double rightRearPosition = 			sensors[SIdx("right rear motor")];

	// Calculate desired positions
	double leftFrontPositionExpected = 		addPositions(leftFrontPosition, 0);
	double leftRearPositionExpected = 		addPositions(leftFrontPosition, phaseSetpoint);
	double rightRearPositionExpected = 		addPositions(leftFrontPosition, phaseSetpoint);
	double rightFrontPositionExpected = 	addPositions(leftFrontPosition, 0);

	// Calculate error between measured desired
	double leftFrontError = 			getPositionError(leftFrontPosition, leftFrontPositionExpected);
	double rightFrontError = 			getPositionError(rightFrontPosition, rightFrontPositionExpected);
	double leftRearError = 				getPositionError(leftRearPosition, leftRearPositionExpected);
	double rightRearError = 			getPositionError(rightRearPosition, rightRearPositionExpected);

	if(ready)
	{
		// Set motor speeds
		motors[MIdx("left front motor")] = 	phaseController(speedSetpoint, leftFrontError);
		motors[MIdx("right front motor")] = phaseController(speedSetpoint, rightFrontError);
		motors[MIdx("left rear motor")] = 	phaseController(speedSetpoint, leftRearError);
		motors[MIdx("right rear motor")] = 	phaseController(speedSetpoint, rightRearError);
	}
	else
	{
		// Go to initial position
		motors[MIdx("left front motor")] = 	phaseController(0, leftFrontError);;
		motors[MIdx("right front motor")] = phaseController(0, rightFrontError);
		motors[MIdx("left rear motor")] = 	phaseController(0, leftRearError);
		motors[MIdx("right rear motor")] = 	phaseController(0, rightRearError);

		if(abs(leftRearError)+abs(rightRearError) < 0.05)
			ready = true;
	}
	// Collect data
	collectData(leftFrontPosition, leftRearPosition, tilt);
}

double SpringyBotPhaseController::addPositions(double position1, double position2) {
	return fmod(position1 + M_PI + position2, 2*M_PI) - M_PI;
}

double SpringyBotPhaseController::getPositionError(double position1, double position2) {
	Vector measured(position1);
	Vector expected(position2);
	return measured.smallestAngle(expected);
}

double SpringyBotPhaseController::phaseController(double setpoint, double error) {
	// TODO: For now a very simple controller
	return setpoint + error;
}

void SpringyBotPhaseController::init(int sensornumber, int motornumber, RandGen* randGen) {
	nSensors = sensornumber;
	nMotors  = motornumber;
	outputFile.open("output.csv", std::ios::app);
	//outputFile << "phase, " << phaseSetpoint << ", vel," << speedSetpoint << std::endl;
	initialised=true;
}

int SpringyBotPhaseController::getSensorNumber() const {
	return nSensors;
}

int SpringyBotPhaseController::getMotorNumber() const {
	return nMotors;
}

bool SpringyBotPhaseController::store(FILE* f) const {
	Configurable::print(f,0);
	return true;
}

bool SpringyBotPhaseController::restore(FILE* f) {
	Configurable::parse(f);
	return true;
}

void SpringyBotPhaseController::setPositionData(Position position) {
    double distance = (position.y > 0 ? 1.0 : -1.0) * sqrt( pow(position.x,2) + pow(position.y,2) );
    //std::cout << "("<< position.x << "," << position.y << ")" << std::endl;
    if (abs(distance) > abs(distanceMax))
    	distanceMax = distance;
}

void SpringyBotPhaseController::collectData(double frontPositionMeasured, double rearPositionMeasured, double tilt) {

	// Register
	if(rearPositionMeasured > rearPositionMax) {
		rearTicksAtMax = ticks_since_reset;
		rearPositionMax = rearPositionMeasured;
	}

	if(tilt > tiltMax) {
		tiltTicksAtMax = ticks_since_reset;
		tiltMax = tilt;
	}

	if(tilt < tiltMin) {
		tiltTicksAtMin = ticks_since_reset;
		tiltMin = tilt;
	}

	// Output
	if(fabs(frontPositionMeasured - frontPositionSaved) > M_PI) {
		if (outputFile.is_open())
			outputFile << 	speedSetpoint << "," <<
							phaseSetpoint << "," <<
							(double)(ticks_since_init*time_per_tick) << "," <<
							(double)(ticks_since_reset*time_per_tick) << ","  <<
							distanceMax << "," <<
							(double)(rearTicksAtMax*time_per_tick) << "," <<
							(double)(tiltTicksAtMin*time_per_tick) << "," <<
							(double)(tiltTicksAtMax*time_per_tick) << "," <<
							(double)(tiltMin) << "," <<
							(double)(tiltMax) <<
							std::endl;
		else
			std::cout << "LocoKit controller: File not open" << std::endl;

		// Reset
		ticks_since_reset = 0;
		rearPositionMax = -10;
		tiltMax = -10;
		rearTicksAtMax = 0;
		tiltTicksAtMax = 0;
		distanceMax = 0;
	}

	// Upkeep
	frontPositionSaved = frontPositionMeasured;
}

Vector::Vector(double angle) {
	_x = cos(angle);
	_y = sin(angle);
}

Vector::Vector(double x, double y) {
	_x = x;
	_y = y;
}

double Vector::angle(Vector vector) {
	double out = 0;
    if (length() && vector.length())
    {
        double tmp = dot(vector) / ( length() * vector.length() );
        if (tmp > 1)
            out = acos(1);
        else if  (tmp < -1)
        	out = acos(-1);
        else
            out = acos(tmp);
    }
    else
        out = 0.0;

    return out;
}

double Vector::length() {
	return sqrt( pow(_x,2) + pow(_y,2) );
}

double Vector::dot(Vector vector) {
	return _x*vector._x + _y*vector._y;
}

double Vector::smallestAngle(Vector vector) {
	double lead = vector.angle( Vector(_x,_y) );
	double lag = angle(vector);
	if(abs(lead) < abs(lag))
		return lead;
	else
		return lag;
}

Vector Vector::rotate(double angle) {
    double new_x = cos(angle)*_x - sin(angle)*_y;
    double new_y = sin(angle)*_x + cos(angle)*_y;
    return Vector(new_x,new_y);

}

