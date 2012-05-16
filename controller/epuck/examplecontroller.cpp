/*
 * This file was created by Tobias Jahn on Tuesday May 01, 2012
 * 
 * This examplecontroller shows the usage of the real epuck interface
 *
 */

#include "examplecontroller.h"

#include <vector>
#include <iostream>
#include <cmath>


ExampleController::ExampleController() : AbstractController("EPuck example controller", "1.0") {
}

ExampleController::~ExampleController() {
}

void ExampleController::init(int sensornumber, int motornumber, RandGen* randGen) {
}


int ExampleController::getSensorNumber() const {
  return motorCount;
}

int ExampleController::getMotorNumber() const {
  return sensorCount;
}

void ExampleController::step(const sensor* sensors, int _sensorCount, motor* motors, int _motorCount) {

       for(int i=0; i<8; i++){
	 (sensors[numOfSensor.IR0+i]>0.5)?motors[numOfMotor.LED0+i]=true:motors[numOfMotor.LED0+i]=false;
       }
       
       double sensval=0;
       for(int i=0; i<8; i++)sensval+=sensors[numOfSensor.IR0+i]/8;
       double a =2;
       double sens_left=  -1./2*(sensors[numOfSensor.IR0]+sensors[numOfSensor.IR1]-0*sensors[numOfSensor.IR2]-2*sensors[numOfSensor.IR3]);
       double sens_right= -1./2*(sensors[numOfSensor.IR6]+sensors[numOfSensor.IR7]-0*sensors[numOfSensor.IR5]-2*sensors[numOfSensor.IR4]);
       
//       if(sensval>0.02) motors[numOfMotor.LED_BODY]=true;       else	 motors[numOfMotor.LED_BODY]=false;
       
       if(sensval>0.6)motors[numOfMotor.SOUND]=2;
       else motors[numOfMotor.SOUND]=0;
       
       motors[numOfMotor.MOTOR_LEFT] += a* (sens_left  -0.*sens_right +0.0*motors[numOfMotor.MOTOR_RIGHT] + 0.01);
       motors[numOfMotor.MOTOR_RIGHT]+= a* (sens_right -0.*sens_left  +0.0*motors[numOfMotor.MOTOR_LEFT]  + 0.01);
       
       double speed_max = 0.8;
       if(motors[numOfMotor.MOTOR_LEFT]>speed_max)motors[numOfMotor.MOTOR_LEFT]=speed_max;
       if(motors[numOfMotor.MOTOR_LEFT]<-speed_max)motors[numOfMotor.MOTOR_LEFT]=-speed_max;
       if(motors[numOfMotor.MOTOR_RIGHT]>speed_max)motors[numOfMotor.MOTOR_RIGHT]=speed_max;
       if(motors[numOfMotor.MOTOR_RIGHT]<-speed_max)motors[numOfMotor.MOTOR_RIGHT]=-speed_max;
       
}

void ExampleController::stepNoLearning(const sensor*, int _sensorCount, motor*, int _motorCount) {

}

bool ExampleController::store(FILE* f) const {
  return false;
}

bool ExampleController::restore(FILE* f) {
  return false;
}