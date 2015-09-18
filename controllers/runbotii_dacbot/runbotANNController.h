/*
 * realRunbotController.h
 *
 *  Created on: 08.03.2014
 *      Author: Johannes Widenka
 */

#ifndef REALRUNBOTCONTROLLER_H_
#define REALRUNBOTCONTROLLER_H_

#include "cgaittransition.h"
#include "cnnet.h"

#include <selforg/abstractcontroller.h>

/// Channel number description
//const int BOOM_ANGLE = 1;
//const int LEFT_FOOT  = 2;   // = 0 .. -4.96v,  2048 = 0V (foot contact the ground) ....
                            //                4096 = + 4.96V (foot off ground)
//const int RIGHT_FOOT = 3;   // channel 3
//const int LEFT_HIP   = 4;
//const int RIGHT_HIP  = 5;
//const int LEFT_KNEE  = 6;
//const int RIGHT_KNEE = 7;



class RunbotANNController : public AbstractController {
  public:
       RunbotANNController(const std::string& name, const std::string& revision);
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
       int steps;
       double ubc;
       double ubc_wabl = 0.0;
       int ubc_time = 100;
       double speed;
       double pos;
       runbot::cNNet* nnet;
       runbot::cGaitTransition* gait;
       valarray<double> actualAD;

       bool initialized = false;


};

#endif /* REALRUNBOTCONTROLLER_H_ */
