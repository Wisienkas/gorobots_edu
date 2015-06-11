/*
 * derivativeTransitionRegister.cpp
 *
 *  Created on: Mar 10, 2015
 *      Author: giuliano
 */

#include <controllers/runbotii_dacbot/derivativeTransitionRegister.h>

derivativeTransitionRegister::derivativeTransitionRegister(double theresholdValueNegToZero, double theresholdValue0ToPos) {
	// TODO Auto-generated constructor stub
    motorValue=0;
    hipValue=0;
	counter=0;
	motorValue0ToPos=0;
	hipValue0ToPos=0;
	thereshold0=theresholdValueNegToZero;
	theresholdPos=theresholdValue0ToPos;
}

derivativeTransitionRegister::~derivativeTransitionRegister() {
	// TODO Auto-generated destructor stub
}

void derivativeTransitionRegister::registerDerivative(double derivativeHip, double signalHip,  double derivativeMotor, double signalMotor,int step, double amplitude)
{


    double size=amplitude/10;

	if((signalHip < 0 && derivativeHip < -2*size) )//reset the counter when hips goes from 0 to neg
		counter=0;


	if(signalMotor > 2*size && signalMotor < 8*size && derivativeMotor >0)
	{
		counter++;
		motorValue0ToPos=step;
	}

	if(signalHip > 0 && derivativeHip > 2*size)
	{
		counter++;
		hipValue0ToPos=step;
	}

	if (signalMotor < -1.5*size && signalMotor > -7.5*size && derivativeMotor >0)//signalMotor > -0.5 motor from neg to zero
	{
		counter++;
		motorValue=step;

	}

	if (signalHip > -2*size && signalHip < 2*size && derivativeHip > 2*size)//hip from neg to zero
	{

		counter++;
		hipValue=step;
	}


}
bool derivativeTransitionRegister::goodPhase()
{
	if (counter > 3 &&  motorValue-hipValue < thereshold0 && motorValue-hipValue > -thereshold0 &&
			  motorValue0ToPos- hipValue0ToPos > theresholdPos &&   motorValue0ToPos- hipValue0ToPos < 3+theresholdPos )//counter ==2
		return true;
	else return false;
}
int derivativeTransitionRegister::getCounter()
{
	return counter;
}

double derivativeTransitionRegister::getActualError()
{
	return motorValue-hipValue;
}

double derivativeTransitionRegister::getHipValues()
{
	return hipValue;
}

double derivativeTransitionRegister::getMotorValues()
{
	return motorValue;
}

bool  derivativeTransitionRegister::checked()
{
	if (counter  == 2)
		return true;
	else return false;
}




