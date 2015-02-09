/*
 * muscleRunbotController.cpp
 *
 *  Created on: 08.03.2014
 *      Author: Johannes Widenka
 */

#include "muscleRunbotController.h"

#define STEPSIZE = 0.01
#define RADIUS = 1;

static double DEGTORAD=M_PI/180.0;

MuscleRunbotController::MuscleRunbotController(const std::string& name, const std::string& revision)
: AbstractController (name,revision){
  steps = 0;
  pos = 0;
  nSensors = 0;
  nMotors = 0;
  simulatedMass = 0.4;
  gait = new runbot::cGaitTransition(runbot::cGaitProfile());
  nnet = new runbot::cNNet((runbot::cGaitProfile*) gait);

  addParameterDef("UBC",&ubc,-0.5,-1.0,1.0,"leaning forward and backwards [1 .. -1]");
  addParameterDef("Mass",&simulatedMass,0.4,0.0,3.0,"simulated mass for muscle model");
  addInspectableValue("speed",&speed,"the speed of runbot, measured in cm/sec");
  initialized = false;
}


void MuscleRunbotController::init(int sensornumber, int motornumber, RandGen* randGen) {
nSensors = sensornumber;
nMotors =  motornumber;
steps = 0;
SigmoidTransitionFunction transF = SigmoidTransitionFunction(100);
MuscleModelConfiguration config = MuscleModelConfiguration(transF);
config.D = 1;         		//0.2
config.Dmin = 0.7;	  		//0.2
config.mass = simulatedMass;//1.5
config.radius = 0.05; 		//1
config.mActFactor = 1.4;	//1.2
config.Kmin = 0.4;	  		//0.2
config.centerAngle = M_PI;
config.groundAngle = M_PI;
config.length = 0.122;
config.timestep = 10;
LKmuscles = new DCControllingVMM(config);
addParameterDef("LKK",&(LKmuscles->K),0.8,0.0,100.0,"Stiffness of the left knee [0 .. 10.0]");
addParameterDef("LKD",&(LKmuscles->D),0.01,0.0,5,"Damping of the left knee [0 .. 10.0]");
RKmuscles = new DCControllingVMM(config);
addParameterDef("RKK",&(RKmuscles->K),0.8,0.0,100,"Stiffness of the right knee [0 .. 10.0]");
addParameterDef("RKD",&(RKmuscles->D),0.01,0.0,5,"Damping of the right knee [0 .. 10.0]");
config.centerAngle = M_PI/2.0;
config.groundAngle = M_PI/2.0;
config.length = 0.142;
config.mActFactor = 1.4; //1.5
LHmuscles = new DCControllingVMM(config);
addParameterDef("LHK",&(LHmuscles->K),0.5,0.0,100,"Stiffness of the left hip [0 .. 1.0]");
addParameterDef("LHD",&(LHmuscles->D),0.01,0.0,5,"Damping of the left hip [0 .. 10.0]");
RHmuscles = new DCControllingVMM(config);
addParameterDef("RHK",&(RHmuscles->K),0.5,0.0,100,"Stiffness of the right hip [0 .. 1.0]");
addParameterDef("RHD",&(RHmuscles->D),0.01,0.0,5,"Damping of the right hip [0 .. 10.0]");

leftMuscles = new MuscleChain(2);
leftMuscles->addMuscle(0,LKmuscles);
leftMuscles->addMuscle(1,LHmuscles);

rightMuscles = new MuscleChain(2);
rightMuscles->addMuscle(0,RKmuscles);
rightMuscles->addMuscle(1,RHmuscles);

addInspectableValue("KLHip",LHmuscles->getCurKAddress(),"current K while walking");
addInspectableValue("DLHip",LHmuscles->getCurDAddress(),"current D while walking");

actualAD = valarray<double>(sensornumber+1);
initialized = true;
}

int MuscleRunbotController::getSensorNumber() const {
  return nSensors;
}

int MuscleRunbotController::getMotorNumber() const {
  return nMotors;
}

void MuscleRunbotController::step(const sensor* sensors, int sensornumber, motor* motors, int motornumber) {
  valarray<double> motorOutput(4);
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


  //musclemodel start
  // execute the following codes in every time step

  //converting foot sensor signals in ratio of weight on this leg
  double Raw_R_fs = (sensors[6]>3000)?0.0:1.0;
  double Raw_L_fs = (sensors[5]>3000)?0.0:1.0;
  double rightLoad = Raw_R_fs * (1 - Raw_L_fs*0.5);
  double leftLoad  = Raw_L_fs * (1 - Raw_R_fs*0.5);

  //VAAM needs radian angles
  double lHAng = sensors[0] * DEGTORAD;
  double rHAng = sensors[1] * DEGTORAD;
  double lKAng = sensors[2] * DEGTORAD;
  double rKAng = sensors[3] * DEGTORAD;



  double LHMusclTor,RHMusclTor,LKMusclTor,RKMusclTor;

  //use the changeable mass for muscle model simulation
  LKmuscles->setMass(simulatedMass);
  RKmuscles->setMass(simulatedMass);
  LHmuscles->setMass(simulatedMass);
  RHmuscles->setMass(simulatedMass);


  /**
   * first step for muscle chain: set all new input values!
   * (if only muscle model is used, without chaining, both steps can be done in one call,
   * but in the chain, each joint also depends on the current state of other joints)
   */

  leftMuscles->setState(0,motorOutput[2],leftLoad,lKAng);
  rightMuscles->setState(0,motorOutput[3],rightLoad,rKAng);
  leftMuscles->setState(1,motorOutput[0],leftLoad,lHAng);
  rightMuscles->setState(1,motorOutput[1],rightLoad,rHAng);

  double tFactor = 0.003; //torquefactor, to  scale muscle models output into the right range (to motor voltage)

  /**
   * second step for muscle chain: get all new output values!
   */
  LKMusclTor = tFactor*leftMuscles->getSignal(0);
  RKMusclTor = tFactor*rightMuscles->getSignal(0);
  LHMusclTor = tFactor*leftMuscles->getSignal(1);
  RHMusclTor = tFactor*rightMuscles->getSignal(1);

  motors[0] = LHMusclTor;
  motors[1] = RHMusclTor;
  motors[2] = LKMusclTor;
  motors[3] = RKMusclTor;
  motors[4] = ubc+ubc_wabl;

  //muscle model end


  //testing stability possible with fast moving upper body component:
  if (steps % ubc_time == 0)
    ubc_wabl *= -1;

  //speed measurement..
  double stepsize = 0.01; //length of control intervalls (both should be obtained automatically somehow..)
  double radius = 1; //armlength in global coordinates
  if ((sensors[7]/10 - pos) < -M_PI)
    speed = speed*0.99+0.01*((sensors[7]/10 + 2*M_PI -pos)*radius/0.01);
  else
    speed = speed*0.99+0.01*((sensors[7]/10 - pos)*radius)/0.01;
  pos = sensors[7]/10;


}



void MuscleRunbotController::stepNoLearning(const sensor* sensors, int number_sensors, motor* motors, int number_motors) {
}

bool MuscleRunbotController::store(FILE* f) const {
}

bool MuscleRunbotController::restore(FILE* f) {
}
