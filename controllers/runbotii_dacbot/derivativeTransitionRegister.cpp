/*
 * derivativeTransitionRegister.cpp
 *
 *  Created on: Mar 10, 2015
 *      Author: giuliano
 */

#include <controllers/runbotii_dacbot/derivativeTransitionRegister.h>

derivativeTransitionRegister::derivativeTransitionRegister() {
	// TODO Auto-generated constructor stub
    for(int i = 0; i<4; i++)
    {
    	motorValues.push_back(0);
    	hipValues.push_back(0);
    }
	motorCount = 0;
	hipCount =0;
}

derivativeTransitionRegister::~derivativeTransitionRegister() {
	// TODO Auto-generated destructor stub
}

void derivativeTransitionRegister::registerDerivative(double derivativeHip, double signalHip,  double derivativeMotor, double signalMotor,int step)
{
	//updating hipsValues
	if (derivativeHip > 0.5)
	{
		if(signalHip > 0.5 )
		{
			hipCount=0;
			hipValues[0]=step;
			hipCount++;
		}
		if(signalHip > -0.5 && signalHip < 0.5)
		{
			hipValues[3]=step;
			hipCount++;
		}
	}

	if (derivativeHip < -0.5)
	{
		if(signalHip < -0.5)
		{
			hipValues[2]=step;
			hipCount++;
		}
		if(signalHip > -0.5 && signalHip < 0.5)
		{
			hipValues[1]=step;
			hipCount++;
		}
	}

	//updating motorValues

	if (derivativeMotor > 0.5)
	{
		if(signalMotor > 0.5 )
		{
			motorCount=0;
			motorValues[0]=step;
			motorCount++;
		}
		if(signalMotor > -0.5 && signalMotor < 0.5)
		{
			motorValues[3]=step;
			motorCount++;

		}
	}

	if (derivativeMotor < -0.5)
	{
		if(signalMotor < -0.5)
		{
			motorValues[2]=step;
			motorCount++;
		}
		if(signalMotor > -0.5 && signalMotor < 0.5)
		{
			motorValues[1]=step;
			motorCount++;
		}

	}



}

std::vector<double> derivativeTransitionRegister::getHipValues()
{
	return hipValues;
}

std::vector<double> derivativeTransitionRegister::getMotorValues()
{
	return motorValues;
}

bool  derivativeTransitionRegister::checked()
{
	if (motorCount  == 4)
		return true;
	else return false;
}




