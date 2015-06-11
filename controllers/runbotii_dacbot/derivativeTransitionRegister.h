/*
 * derivativeTransitionRegister.h
 *
 *  Created on: Mar 10, 2015
 *      Author: giuliano
 */

#ifndef CONTROLLERS_RUNBOTII_DACBOT_DERIVATIVETRANSITIONREGISTER_H_
#define CONTROLLERS_RUNBOTII_DACBOT_DERIVATIVETRANSITIONREGISTER_H_

#include <vector>
#include <math.h>
#include <iostream> //for plotting
#include <fstream> //plotting
class derivativeTransitionRegister {
public:
	derivativeTransitionRegister(double theresholdValueNegToZero, double theresholdValue0ToPos);
	virtual ~derivativeTransitionRegister();
	void registerDerivative(double derivativeHip, double signalHip,  double derivativeMotor, double signalMotor,int step, double amplitude);
    int counter;
    double getActualError();
    bool checked();
    double getHipValues();
    double getMotorValues();
    int getCounter();
    bool goodPhase();
private:

    double motorValue, hipValue, thereshold0,theresholdPos, motorValue0ToPos, hipValue0ToPos;
};

#endif /* CONTROLLERS_RUNBOTII_DACBOT_DERIVATIVETRANSITIONREGISTER_H_ */
