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
class derivativeTransitionRegister {
public:
	derivativeTransitionRegister();
	virtual ~derivativeTransitionRegister();
	void registerDerivative(double derivativeHip, double signalHip,  double derivativeMotor, double signalMotor,int step);
    int motorCount, hipCount;
    double getMagnitude();
    bool checked();
    std::vector<double> getHipValues();
    std::vector<double> getMotorValues();
private:

    std::vector<double> motorValues, hipValues;
};

#endif /* CONTROLLERS_RUNBOTII_DACBOT_DERIVATIVETRANSITIONREGISTER_H_ */
