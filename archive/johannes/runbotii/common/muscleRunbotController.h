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

#include <selforg/abstractcontroller.h>
#include "vaams-framework/dccontrollingvaams.h"
#include "vaams-framework/musclechain.h"

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

       DCControllingVAAMs *RHmuscles;		//modelled muscles for each joint..
       DCControllingVAAMs *LHmuscles;
       DCControllingVAAMs *RKmuscles;
       DCControllingVAAMs *LKmuscles;

       	   	   	   	   	   	   	   	   //muscle chains that handle the communication for muscles depending on each other
       MuscleChain *leftMuscles;
       MuscleChain *rightMuscles;

       valarray<double> actualAD;		//array, used for mapping the sensor array to the right order

       bool initialized = false;

};

#endif /* MUSCLERUNBOTCONTROLLER_H_ */
