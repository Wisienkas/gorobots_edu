/*
 * muscleRunbotController.h
 *
 *  Created on: 08.03.2014
 *      Author: Johannes Widenka
 */

#ifndef MUSCLERUNBOTCONTROLLER_H_
#define MUSCLERUNBOTCONTROLLER_H_

#include "cgaittransition.h"
#include "cnnet.h"
#include "plastic.h"
#include "lowPassfilter.h"
#include <vector>
#include <cmath>
#include <controllers/runbotii_dacbot/shiftregister.h>
#include <controllers/runbotii_dacbot/derivativeTransitionRegister.h>
//CONTROLLERS_RUNBOTII_DACBOT_LOWPASSFILTER_H_

//#include "doublefann.h"
//#include "floatfann.h"


#include <iostream> //for plotting
#include <fstream> //plotting

#include <selforg/abstractcontroller.h>
//#include "VAAMlib/dccontrollingvmm.h"
//#include "VAAMlib/musclechain.h"

#include "utils/vaam-library/musclechain.h"
#include "utils/vaam-library/dccontrollingvmm.h"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

template <typename T> int abs(T val) {
    return (T(0) < val)?-val:val;
}

/// Channel number description
const int BOOM_ANGLE = 1;
const int LEFT_FOOT  = 2;   // = 0 .. -4.96v,  2048 = 0V (foot contact the ground) ....
                            //                4096 = + 4.96V (foot off ground)
const int RIGHT_FOOT = 3;   // chanel 3
const int LEFT_HIP   = 4;
const int RIGHT_HIP  = 5;
const int LEFT_KNEE  = 6;
const int RIGHT_KNEE = 7;


class MuscleRunbotController : public AbstractController {
  public:
    MuscleRunbotController(const std::string& name, const std::string& revision);
    /** initialisation of the controller with the given sensor/ motornumber
         Must be called before use. The random generator is optional.
         */
       virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

       /** @return Number of sensors the controller
         was initialised with or 0 if not initialised */
       virtual int getSensorNumber() const;

       /** @return Number of motors the controller
         was initialised with or 0 if not initialised */
       virtual int getMotorNumber() const;

       /** performs one step.
         Calculates motor commands from sensor inputs.
         @param sensors sensors inputs scaled to [-1,1]
         @param sensornumber length of the sensor array
         @param motors motors outputs. MUST have enough space for motor values!
         @param motornumber length of the provided motor array
         */
       virtual void step(const sensor* sensors, int sensornumber,
           motor* motors, int motornumber);


       /** performs one step without learning.
         @see step
         */
       virtual void stepNoLearning(const sensor* sensors, int number_sensors,
           motor* motors, int number_motors);
       /** stores the object to the given file stream (binary).
       */
       virtual bool store(FILE* f) const;

       /** loads the object from the given file stream (binary).
       */
       virtual bool restore(FILE* f);

     private:
       int nSensors;
       int nMotors;
       int steps;		// counter for controll steps
       double ubc;		// position of the upper body component (-1..+1)
       double speed;	// current speed of the robot
       double pos;		// current position of the robot
       double simulatedMass;	//simulated mass of the robot, used by all modelled muscles
       double ubc_wabl = 0.0;	//movement of the UBC around its position variable "ubc"
       	   	   	   	   	   	   	//wabbling from ubc-ubc_wabl to ubc+ubc_wabl
       int ubc_time = 100; 		// sec/100   -   defines the time where the ubc changes its movement direction
       runbot::cNNet* nnet;		// ANN controlling the movement of the robot
       runbot::cGaitTransition* gait;	// gait parameter for the ANN

       DCControllingVMM *RHmuscles;		//modelled muscles for each joint..
       DCControllingVMM *LHmuscles;
       DCControllingVMM *RKmuscles;
       DCControllingVMM *LKmuscles;

       	   	   	   	   	   	   	   	   //muscle chains that handle the communication for muscles depending on each other
       MuscleChain *leftMuscles;
       MuscleChain *rightMuscles;

       valarray<double> actualAD;		//array, used for mapping the sensor array to the right order

       bool initialized = false;
       //giuliano

       std::ofstream hipPlot;//plot
       std::ofstream cpgPlot;
       std::ofstream train;
       plastic *cpg;
       plastic *feet_cpg;
       plastic *knee_cpg;
       double sensorFeedback;
       double feetFeedback;
       lowPass_filter* filter;
       lowPass_filter* knee_filter;
       std::vector<double> leftDerivativeVector, rightDerivativeVector, motor0DerivativeVector, leftHipDerivativeVector;
       double cpg_right_hip=0;
       double cpg_left_hip=0;

       double cpg_left_knee=0;
       double cpg_right_knee=0;

       double cpg_signal_right;
       double cpg_signal_left;
       double perturbation;
       //giuliano
       std::vector<double> generateCPGhips(double signal, double derivative, double oscillation);
       std::vector<double> generateCPGknee(double signal, double derivative, double oscillation, double value);
       double generateLeftKnee(double signal, double derivative, double oscillation, double value);
       double getAbsol(double a, double b);
       bool cpgControl=false;
       int count=0;
       int goodCounter=0;
       int common_points=0;

       double getDelay(double value,std::vector<double> &shift_register);
       std::vector<double> shift;
       shift_register *leftKneeDelayed,*rightKneeDelayed,*leftHipDelayed, *rightHipDelayed;
       derivativeTransitionRegister *checkWave;
       double createPerturbation(int start, int end, double perturbation, int step, bool CpgControl);
       double changeRange(double oldMin, double oldMax, double newMin, double newMarx, double value);
};

#endif /* MUSCLERUNBOTCONTROLLER_H_ */
