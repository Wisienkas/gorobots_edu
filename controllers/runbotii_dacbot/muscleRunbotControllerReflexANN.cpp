/*
 * muscleRunbotController.cpp
 *
 *  Created on: 08.03.2014
 *      Author: Johannes Widenka
 */

//#include "muscleRunbotController.h"

#include "muscleRunbotControllerReflexANN.h"

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

//Giuliano
hipPlot.open("/home/giuliano/Documents/thesis/plots/hips.dat");
cpgPlot.open("/home/giuliano/Documents/thesis/plots/cpg.dat");
train.open("/home/giuliano/Documents/thesis/plots/train.data");



motor0DerivativeVector.push_back(0);
motor0DerivativeVector.push_back(0);

leftHipDerivativeVector.push_back(0);
leftHipDerivativeVector.push_back(0);

leftDerivativeVector.push_back(0);
leftDerivativeVector.push_back(0);

rightDerivativeVector.push_back(0);
rightDerivativeVector.push_back(0);

//cpg= new plastic(0.2,0.2,0.2,0.04*2*3.14,1.01);
cpg= new plastic(0.2,0.2,0.2,0.04*2*3.14,1.01);


feet_cpg= new plastic(0.2,0.2,0.2,0.04*2*3.14,1.01,0.01);
knee_cpg= new plastic(0.2,0.2,0.2,0.05*2*3.14,1.01,0.03);

//checkWave = new derivativeTransitionRegister();


filter= new lowPass_filter(0.2);
knee_filter=new lowPass_filter(0.1);
leftHipDelayed=new shift_register(0);
rightHipDelayed= new shift_register(0);

leftKneeDelayed=new shift_register(6);
rightKneeDelayed=new shift_register(6);


//Giuliano

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

double MuscleRunbotController::getAbsol(double a, double b)
{
	double result=a-b;
	if (result>0)
		return result;
	else return -result;

}


double MuscleRunbotController::getDelay(double value,std::vector<double> &shift_register )
{
   double temp=shift_register.at(0);
   for(int i=shift_register.size()-2;i > -1;i--)
	   shift_register[i]=shift_register.at(i+1);
   shift_register[shift_register.size()-1] = value;
   std::cout<< "delayyyyyyyyy" <<temp;
   return temp;

}
std::vector<double> MuscleRunbotController::generateCPGhips(double signal, double derivative,double oscillation)
{
	std::vector<double> result;
	double right, left;
	if (signal> oscillation)
	  {

		if (derivative >= 0)
		{
			left=2.18;
			right=-2.18;


		}

		else
		{
			left=0;
			right=0;
		}

	  }

	  if (signal <= oscillation)
	  {

		  if (derivative < 0)
		  {
			left=-2.18;
			right=2.18;
		  }

		  else
		  {
			left=0;
			right=0;

		  }

	  }

	  result.push_back(left);
	  result.push_back(right);

	  return result;


}
double MuscleRunbotController::changeRange(double oldMin, double oldMax, double newMin, double newMax, double value)
{

	return ((value-oldMin)/(oldMax-oldMin)) * (newMax-newMin) + newMin;

}
std::vector<double> MuscleRunbotController::generateCPGknee(double signal, double derivative, double oscillation,double value)//0 left, 1 right
{
	double left,right;
	std::vector<double> result;

	if(signal > oscillation)
	{
		right=value;
		if(derivative >=0)
			left=-value;
		else //else left=1.77;
			left = value;

	}

	if(signal <= oscillation)
	{
		left=value;
		if(derivative >=0)
			right=value;
		else right=-value;
	}

	result.push_back(left);
	result.push_back(right);

	return result;
}
double MuscleRunbotController::generateLeftKnee(double signal, double derivative, double oscillation, double value)
{
	double temp;
	if(signal >= oscillation && derivative >0)
		temp = -signal;
	else temp=value;

	return temp;
}
double MuscleRunbotController::createPerturbation(int start, int end, double perturbation, int step,bool CpgControl)
{

	if (CpgControl == true)
		return 0;
	else
	{

	if (step >= start && step < end)
		return perturbation;
	else return 0;
	}
}

void MuscleRunbotController::step(const sensor* sensors, int sensornumber, motor* motors, int motornumber) {
  valarray<double> motorOutput(4);
  steps++;

  std::cout << steps << " ";

  actualAD[LEFT_HIP] = sensors[0];
  actualAD[RIGHT_HIP] = sensors[1];
  actualAD[LEFT_KNEE] = sensors[2];
  actualAD[RIGHT_KNEE] = sensors[3];
  actualAD[BOOM_ANGLE] = sensors[4];
  actualAD[LEFT_FOOT] = sensors[5];
  actualAD[RIGHT_FOOT] = sensors[6];
  actualAD[0] = steps;

  motorOutput=nnet->update(actualAD,steps);

  leftHipDerivativeVector.push_back(0);
  motor0DerivativeVector[0]=motor0DerivativeVector.at(1);
  motor0DerivativeVector[1]=motorOutput[0];
  double derivativeMotor0= motor0DerivativeVector[1]-motor0DerivativeVector[0];

  double left_foot_sensor = (sensors[6]>3000)?0.0:1.0;
  double right_foot_sensor = (sensors[5]>3000)?0.0:1.0;

  double filteredHipAngleCpgRange = filter->update(actualAD[LEFT_HIP]);//low pass filtering the hip feedback
  filteredHipAngleCpgRange = changeRange(60,120,-0.2,0.2,filteredHipAngleCpgRange);//( (filteredHipAngle - 60)/60) * 0.4 -0.2; // changing range from [-60:120] to [-0.2:0.2]

  double hipCpgPerturbation = createPerturbation(2000, 4000, filteredHipAngleCpgRange, steps,false);

  //if(steps < 4000 || steps > 15000)
//	  hipCpgPerturbation=filteredHipAngleCpgRange;
 // else hipCpgPerturbation=0;
  cpg->update(hipCpgPerturbation);

  double leftHip, rightHip, leftKnee, rightKnee;
  std::vector<double> hips,knees;

  leftDerivativeVector[0]=leftDerivativeVector.at(1);
  leftDerivativeVector[1]=cpg->getOut1();
  double derivativeLeftKnee= leftDerivativeVector[1]-leftDerivativeVector[0];

  hips=generateCPGhips(cpg->getOut1(), derivativeLeftKnee,0);
  leftHip=hips[0];
  rightHip=hips[1];

  leftHipDerivativeVector[0]=leftHipDerivativeVector.at(1);
  leftHipDerivativeVector[1]=leftHip;
  double derivativeLeftHip= leftHipDerivativeVector[1]-leftHipDerivativeVector[0];



  knees= generateCPGknee(cpg->getOut1(), derivativeLeftKnee, 0,1.77);
  leftKnee=knees[0];
  rightKnee=knees[1];

  //if (steps > 2400)
 // checkWave->registerDerivative(derivativeLeftHip, leftHip, derivativeMotor0, motorOutput[0],steps);
 // train << steps <<" " <<  checkWave->checked() << " " << checkWave->getMotorValues().at(0) << " " << checkWave->getMotorValues().at(1) << " " << checkWave->getMotorValues().at(2) << " " << checkWave->getMotorValues().at(3) << std::endl;


  if(steps >= 3800 && getAbsol(leftHip,motorOutput[0]) < 0.01 && count == 0 && left_foot_sensor*right_foot_sensor >0 && leftHip > 0)
   {
 	  cpgControl=true;
 	  std::cout<< "changing control at" << steps;
 	  count++;
   }
  //right side
 if (cpgControl == false)
	 std::cout << "REFLEXIVE CONTROL" << std::endl;
 else std::cout << "CENTRAL PATTERN GENERATOR CONTROL" << std::endl;

 if (cpgControl == false)
	 cpgPlot << steps  <<" "<< "0"<<std::endl;
  else cpgPlot << steps  <<" "<< "1"<<std::endl;

 double left;

  if(cpgControl==false)
  {

  left=motorOutput[0];
  motors[0] = motorOutput[0];//LHMusclTor;
  motors[1] = motorOutput[1];//RHMusclTor;
  motors[2] = motorOutput[2];//LKMusclTor;
  motors[3] = motorOutput[3];//RKMusclTor;
  motors[4] = ubc+ubc_wabl;
  }
  else
  {
  left=leftHip;
  motors[0] = leftHip;//LHMusclTor;
  motors[1] = rightHip;//RHMusclTor;
  motors[2] = leftKnee;//LKMusclTor;
  motors[3] = rightKnee;//RKMusclTor;
  motors[4] = ubc+ubc_wabl;

  }

  hipPlot << steps <<  " " << cpgControl  << " " << left << " "<<leftHip << " " << motorOutput[0] << " " << rightHip  << " " << leftKnee <<" "<<motorOutput[2]<< " "<<rightKnee << " " << motorOutput[3]<<" " << cpg->getOut1()<< " " << cpg->getFrequency()<<" " << hipCpgPerturbation << std::endl;

  //cpgPlot << steps  <<" "<< hipCpgPerturbation<< " "<<cpg->getOut0() << " " << cpg->getOut1() <<std::endl;


  //cpgPlot << steps << " " << speed << " " << n  << " " << actualAD[LEFT_HIP]  << " "<<motorOutput[0]<<" " << cpg_left_hip<<" "<<cpg_signal_left << " " << cpg->getOut1()<< std::endl;



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
