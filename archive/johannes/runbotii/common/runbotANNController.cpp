/*
 * realRunbotController.cpp
 *
 *  Created on: 08.03.2014
 *      Author: Johannes Widenka
 */

#include "runbotANNController.h"
#include "muscleRunbotController.h"

#define STEPSIZE = 0.01
#define RADIUS = 1;

RunbotANNController::RunbotANNController(const std::string& name, const std::string& revision)
: AbstractController (name,revision){
  steps = 0;
  pos = 0;
  nSensors = 0;
  nMotors = 0;
  gait = new runbot::cGaitTransition(runbot::cGaitProfile(85,95,110,175,1.6,1.55));
  nnet = new runbot::cNNet((runbot::cGaitProfile*) gait);
  addParameterDef("UBC",&ubc,-0.8,-1.0,1.0,"leaning forward and backwards [1..-1]");
  addInspectableValue("speed",&speed,"the speed of runbot, measured in cm/sec");
  initialized = false;
}


void RunbotANNController::init(int sensornumber, int motornumber, RandGen* randGen) {
nSensors = sensornumber;
nMotors =  motornumber;
steps = 0;
actualAD = valarray<double>(sensornumber+1);
initialized = true;
}

int RunbotANNController::getSensorNumber() const {
  return nSensors;
}

int RunbotANNController::getMotorNumber() const {
  return nMotors;
}

void RunbotANNController::step(const sensor* sensors, int sensornumber, motor* motors, int motornumber) {
  valarray<double> motorOutput(4);
  //map values to expectations of real controller
  steps++;
  actualAD[LEFT_HIP] = sensors[0];
  actualAD[RIGHT_HIP] = sensors[1];
  actualAD[LEFT_KNEE] = sensors[2];
  actualAD[RIGHT_KNEE] = sensors[3];
  actualAD[BOOM_ANGLE] = sensors[4];
  actualAD[LEFT_FOOT] = sensors[5];
  actualAD[RIGHT_FOOT] = sensors[6];
  actualAD[0] = steps;
  motorOutput=nnet->update(actualAD,steps);
  //no mapping necessary?

  for (int i = 0; i < 2; i++)
  motors[i] = 1.0*(motorOutput[i]);
  for (int i = 2; i < 4; i++)
  motors[i] = 1.2*(motorOutput[i]);
  motors[4] =  ubc+ubc_wabl; //manually set upper body component

  if (steps % ubc_time == 0)
    ubc_wabl *= -1;

  //speed measurement..
  double stepsize = 0.01; //length of control intervalls (both should be obtained automatically..)
  double radius = 1; //armlength in global coordinates
  if ((sensors[7]/10 - pos) < -M_PI)
    speed = speed*0.99+0.01*((sensors[7]/10 + 2*M_PI -pos)*radius/0.001);
  else
    speed = speed*0.99+0.01*((sensors[7]/10 - pos)*radius)/0.001;
  pos = sensors[7]/10;


}

void RunbotANNController::stepNoLearning(const sensor* sensors, int number_sensors, motor* motors, int number_motors) {
}

bool RunbotANNController::store(FILE* f) const {
}

bool RunbotANNController::restore(FILE* f) {
}
