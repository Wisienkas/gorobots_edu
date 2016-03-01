/*
 * DynamicCpg.h
 *
 *  Created on: Apr 2, 2015
 *      Author: Giuliano Di Canio
 */

#ifndef CONTROLLERS_RUNBOTII_DACBOT_DYNAMICCPG_H_
#define CONTROLLERS_RUNBOTII_DACBOT_DYNAMICCPG_H_


#include <controllers/runbotii_dacbot/derivativeTransitionRegister.h>
#include "plastic.h"
#include "lowPassfilter.h"
#include <vector>
#include <cmath>
#include <string>
#include <controllers/runbotii_dacbot/DynamicVector.h>
#include <controllers/runbotii_dacbot/shiftregister.h>

class DynamicCpg {
public:
	DynamicCpg(double cpgInitialFrequency);

	virtual ~DynamicCpg();

private:
	int countPerturbation;
	DynamicVector *dynRegister;
	std::vector<double> derHip;
	plastic *cpg;
	int delayValue;
	lowPass_filter *filterFeedback;
	derivativeTransitionRegister *checkSignal;
	shift_register *feedbackDelay;
	std::ofstream cpgPlot;
	double maxHip=0, maxKnee=0;
	std::vector<double> maxKneeDer;
	double phaseDelay;
	std::vector<double> motor0Derivative, cpgOut1Derivative, signal0Derivative,signal2Derivative, hipAngleDerivative, maxDer;
	std::vector<double> systemVectorFreq,der, derOut1,derOut2,shiftVector;
	double amplitudeMotor0, amplitudeMotor2, maxFeedback, minFeedback;
	double cpgSignal0, cpgSignal1, cpgSignal2, cpgSignal3;
	bool enableCpgController;
	int counter, step, countder, countDelay=0;
	bool countDelay1, countDelay2;
	int counterDelay1Value, counterDelay2Value;
	int enable;
	bool single, delayFiltered, delayFeedback;
	int counterDelayFeedback, counterDelayFiltered;
    std::vector<double> delay;
	double changeRange(double oldMin, double oldMax, double newMin, double newMarx, double value);
    void updateCpg(double perturbation);
    std::vector<double> generateCpgHipsMultiple(double signal, double derivative,double oscillation, double value);
    std::vector<double> generateCpgKneeMultiple(double signal, double derivative, double oscillation, double value);
    double generateHip(double signal, double derivative,double oscillation, double value);
    double generateKnee(double signal, double derivative,double oscillation, double valueHip, double valueKnee);
    std::vector<double> getMaxMinFeedback(double feedback, double max, double min);
    double getAmplitudeHips(double signal, double max);
    double getAmplitudeKnee(double signal_knee, double max);
    double getAbsol(double a, double b);
    double getShiftDelay(double out1, double out2, int step, double fequency);
public:
    std::vector<double> generateOutputOneLeg(double feedback, double motor0, double motor1, double stepValue);
    std::vector<double> generateOutputTwoLegThereshold(double feedback, double motor0, double motor1, double stepValue);
    void setEnable(bool externalCondition,double motor0, double leftFoot, double rightFoot);
    int getEnable();
    std::vector<double> getGeneratedMotorsSingle();
    std::vector<double> getGeneratedMotorsDouble();
    plastic* getCpg();

};

#endif /* CONTROLLERS_RUNBOTII_DACBOT_DYNAMICCPG_H_ */
