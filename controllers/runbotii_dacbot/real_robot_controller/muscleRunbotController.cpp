
#include "muscleRunbotController.h"
using namespace matrix;
using namespace std;

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

  newGait=new runbot::cGaitProfile(78,97,115,175,2,1.5);
  nnetTwo= new runbot::cNNet((runbot::cGaitProfile*)newGait);

  gait3=new runbot::cGaitProfile(78,115,115,175,2,2.7);
  nnet3=new runbot::cNNet((runbot::cGaitProfile*)gait3);


  addParameterDef("UBC",&ubc,-0.5,-1.0,1.0,"leaning forward and backwards [1 .. -1]");
  addParameterDef("Mass",&simulatedMass,0.4,0.0,3.0,"simulated mass for muscle model");
  addInspectableValue("speed",&speed,"the speed of runbot, measured in cm/sec");
  addInspectableValue("motorvolt_hl",&nnet->motorvolt_hl,"motorvolt_hl");
  addInspectableValue("motorvolt_hr",&nnet->motorvolt_hr,"motorvolt_hr");
  addInspectableValue("motorvolt_kl",&nnet->motorvolt_kl,"motorvolt_kl");
  addInspectableValue("motorvolt_kr",&nnet->motorvolt_kr,"motorvolt_kr");
  addInspectableValue("u_hl_em",&nnet->u_hl_em,"u_hl_em");
  addInspectableValue("u_hl_fm",&nnet->u_hl_fm,"u_hl_fm");
  addInspectableValue("u_hr_em",&nnet->u_hr_em,"u_hr_em");
  addInspectableValue("u_hr_fm",&nnet->u_hr_fm,"u_hr_fm");
  addInspectableValue("u_kl_em",&nnet->u_kl_em,"u_kl_em");
  addInspectableValue("u_kl_fm",&nnet->u_kl_fm,"u_kl_fm");
  addInspectableValue("u_kr_em",&nnet->u_kr_em,"u_kr_em");
  addInspectableValue("u_kr_fm",&nnet->u_kr_fm,"u_kr_fm");

  addInspectableValue(" u_kr_es",&nnet-> u_kr_es," u_kr_es");
  addInspectableValue(" u_kl_es",&nnet-> u_kl_es," u_kl_es");
  addInspectableValue("  u_kr_ei",&nnet->  u_kr_ei,"  u_kr_ei");




  addInspectableValue("state_u_kr_em",&nnet->state_u_kr_em,"state_u_kr_em");
  addInspectableValue("state_u_kr_fm",&nnet->state_u_kr_fm,"state_u_kr_fm");
  addInspectableValue("state_u_kl_em",&nnet->state_u_kl_em,"state_u_kr_em");
  addInspectableValue("state_u_kl_fm",&nnet->state_u_kl_fm,"state_u_kr_fm");

  addInspectableValue("state_motorvolt_kl",&nnet->state_motorvolt_kl ,"state_motorvolt_kl");
  addInspectableValue("state_motorvolt_kr",&nnet->state_motorvolt_kr,"state_motorvolt_kr");
  addInspectableValue("state_motorvolt_hl",&nnet->state_motorvolt_hl,"state_motorvolt_hl");
  addInspectableValue("state_motorvolt_hr",&nnet->state_motorvolt_hr,"state_motorvolt_hr");


  addInspectableValue("u_al",&nnet->u_al ,"u_al");
  addInspectableValue("u_ar",&nnet->u_ar,"u_ar");
  addInspectableValue("u_gl",&nnet->d_u_gl,"d_u_gl");
  addInspectableValue("u_gr",&nnet->d_u_gr,"d_u_gr");


  addInspectableValue("angle_hl_now",&nnet->angle_hl_now,"angle_hl_now");
  addInspectableValue("angle_hr_now",&nnet->angle_hr_now,"angle_hr_now");
  addInspectableValue("angle_hl_lowpass",&nnet->angle_hl_low_pass,"angle_hl_low_pass");
  addInspectableValue("angle_hr_lowpass",&nnet->angle_hr_low_pass,"angle_hr_low_pass");

  addInspectableValue("angle_kl_now",&nnet->angle_kl_now,"angle_kl");
  addInspectableValue("angle_kr_now",&nnet->angle_kr_now,"angle_kr");
  addInspectableValue("angle_kl_lowpass",&nnet->angle_kl_low_pass,"angle_kl_low_pass");
  addInspectableValue("angle_kr_lowpass",&nnet->angle_kr_low_pass,"angle_kr_low_pass");


  /*
  std::cout << "u_gr>>>>" << u_gr << std::endl;
  std::cout << "u_gl>>>>" << u_gl << std::endl;
  std::cout << "u_al>>>>" << u_al << std::endl;
  std::cout << "u_ar>>>>" << u_ar << std::endl;

  std::cout << "u_kl_em>>>>" << u_kl_em << std::endl;
  std::cout << "u_kl_fm>>>>" << u_kl_fm << std::endl;
  std::cout << "u_kr_em>>>>" << u_kr_em << std::endl;
  std::cout << "u_kr_fm>>>>" << u_kr_fm << std::endl;


  std::cout << "u_hl_em>>>>" << u_hl_em << std::endl;
  std::cout << "u_hl_fm>>>>" << u_hl_fm << std::endl;
  std::cout << "u_hr_em>>>>" << u_hr_em << std::endl;
  std::cout << "u_hr_fm>>>>" << u_hr_fm << std::endl;


  std::cout << "motorvolt_kl>>>>" << motorvolt_kl << std::endl;
  std::cout << "motorvolt_kr>>>>" << motorvolt_kr << std::endl;
  std::cout << "motorvolt_hl>>>>" << motorvolt_hl << std::endl;
  std::cout << "motorvolt_hr>>>>" << motorvolt_hr << std::endl;
  */

  initialized = false;
}


void MuscleRunbotController::init(int sensornumber, int motornumber, RandGen* randGen) {
nSensors = sensornumber;
nMotors =  motornumber;
steps = 0;

hipPlot.open("/home/dacbot/Documents/feedback.dat");//change this to your directory



motor0DerivativeVector.push_back(0);
motor0DerivativeVector.push_back(0);

leftHipDerivativeVector.push_back(0);
leftHipDerivativeVector.push_back(0);

leftDerivativeVector.push_back(0);
leftDerivativeVector.push_back(0);

rightDerivativeVector.push_back(0);
rightDerivativeVector.push_back(0);

systemFrequencyVector.push_back(0);
systemFrequencyVector.push_back(0);

freq.push_back(0);
freq.push_back(0);
shiftVector.push_back(0);
shiftVector.push_back(0);

freqDeriv.push_back(0);
freqDeriv.push_back(0);

stepFreq.push_back(0);
stepFreq.push_back(0);


frequencySystem=0;

DinLeft= new DynamicCpg(0.04);//0.04
DinRight= new DynamicCpg(0.04);//0.04


filter= new lowPass_filter(0.2);
phase=new shift_register(3);//4
rightHipDelayed= new shift_register(0);

leftKneeDelayed=new shift_register(6);
rightKneeDelayed=new shift_register(4);



/*
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
*/
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
	///IMPORTANT//
	//This function can either call the Adaptive CPG-based controller with one CPG or the CPG-based controller with
    //two CPGs. Set the variable (in muscleRunbotController.cpp) oneCPG=true for controller with one cpg
	//and oneCPG=false for controller with two CPGs, an if-else statement will switch between the two options


	if(oneCPG==true)//using only one CPG in the controller
{

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

	  double enable;
	  //std::cout << steps<< std::endl;

	  double sensora;
	  
		double left_foot_sensor = (actualAD[LEFT_FOOT]>3000)?0.0:1.0;
	  double right_foot_sensor = (actualAD[RIGHT_FOOT]>3000)?0.0:1.0;

	//std::cout<<"sensors left : "<<left_foot_sensor<<"\n"<<std::endl;
	//std::cout<<"sensors right : "<<right_foot_sensor<<"\n"<<std::endl;


      //Used to generate 1 gait with reflexive nn
	  //motorOutput=nnet->update(actualAD,steps);

	  //This is used to generate different gaits with the reflexive NN, change according to your experiment
	 // if(steps < 5000)
	  	motorOutput=nnet->update(actualAD,steps);
	 // else if(steps >= 5000 && steps <7000)
	  //  motorOutput=nnetTwo->update(actualAD,steps);
	  //else  motorOutput=nnet3->update(actualAD,steps);




	  sensora=actualAD[LEFT_HIP];// feedback

	  //This is to simulate the feedback, you can cut the feedback to the CPG or you can activate it in different periods

	  // if((steps>3000 && steps<5000) || (steps >7000 && steps < 9000) || (steps >11000 && steps < 13000) || (steps >15000 && steps < 19000))
		// sensora=0;

	  // if(steps > 4000)
	    //  sensora=0;



	   std::vector<double> sign = DinLeft->generateOutputTwoLegThereshold(sensora ,motors[0],motors[2], steps );//generates motor signals
      DinLeft->setEnable(false,motors[0],  left_foot_sensor ,right_foot_sensor);// generates enable output
      enable=DinLeft->getEnable();//set enable
      //std::cout << " enable: " << enable << std::endl;

      //wrtie to file, change directory
       
          hipPlot << steps <<" "<<sensors[0] <<" "<<sensors[1] <<" "<<sensors[2] <<" "<<sensors[3] <<" "<<DinLeft->getCpg()->getFrequency()*2.1/0.015<< " "<<sign[0]<<" "<<motorOutput[0] << " " << motorOutput[2]<<" " << motorOutput[3]  <<
           	  " "<< actualAD[LEFT_HIP]<<" "<<actualAD[RIGHT_HIP]<<" " <<actualAD[LEFT_KNEE]<<" "<<
       	  	  actualAD[RIGHT_KNEE]<<" "<<actualAD[LEFT_FOOT]<< " " << actualAD[RIGHT_FOOT]<<std::endl ;
          


      // Depending on the enable, either reflexive signals or CPG-based are sent to the motors
      /*motors[0] = motorOutput[0]*!enable + sign.at(0)*enable ;//LHMusclTor;
	  motors[1] = motorOutput[1]*!enable + sign.at(1)*enable ;//RHMusclTor;
	  motors[2] = motorOutput[2]*!enable + sign.at(2)*enable ;//LKMusclTor;
	  motors[3] = motorOutput[3]*!enable + sign.at(3)*enable ;//RKMusclTor;
	  motors[4] = ubc+ubc_wabl;*/


	motors[0] = motorOutput[0] ;//LHMusclTor;
	  motors[1] = motorOutput[1];//RHMusclTor;
	  motors[2] = motorOutput[2] ;//LKMusclTor;
	  motors[3] = motorOutput[3];//RKMusclTor;
	  motors[4] = ubc+ubc_wabl;


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



	/////////////////////// two CPGs///////////////////////////



	else //using two cpgs
{

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

			  double enable;
			  //std::cout << steps<< std::endl;

			  double sensora;
			  double left_foot_sensor = (sensors[6]>3000)?0.0:1.0;
			  double right_foot_sensor = (sensors[5]>3000)?0.0:1.0;



			  motorOutput=nnet->update(actualAD,steps);
			  //generating different gaits
/*
			  if(steps < 500)
			  motorOutput=nnet->update(actualAD,steps);
			  else
		      motorOutput=nnetTwo->update(actualAD,steps);
*/
			  sensora=actualAD[LEFT_HIP];

			  // Left CPG controlling left leg
			  std::vector<double> LeftLeg = DinLeft->generateOutputOneLeg(actualAD[LEFT_HIP] ,motors[0],motors[2], steps );
		      DinLeft->setEnable(steps>2000,motors[0],  left_foot_sensor ,right_foot_sensor);
		      double LeftEnable=DinLeft->getEnable();


		      // Right CPG controlling right leg
              std::vector<double> RightLeg = DinRight->generateOutputOneLeg(actualAD[RIGHT_HIP],motors[1],motors[3], steps );
              DinRight->setEnable(steps>2000,motors[1],  left_foot_sensor ,right_foot_sensor);
		      double RightEnable=DinRight->getEnable();

		      controllerEnable=LeftEnable*RightEnable;//enable set to 1 when both right and left enable are set to 1


		      //std::cout << "steps " << steps << "   leftEnable" << LeftEnable <<"   righttEnable" << RightEnable << "   Enable" <<controllerEnable << std::endl;



		      if(steps > 500)
		    	  hipPlot << steps << " "<< LeftEnable<<" "<<RightEnable<< " " <<LeftLeg.at(0)<< " " <<RightLeg.at(0) <<" " << DinLeft->getCpg()->getFrequency()  << std::endl ;

		    	  //hipPlot << steps << " "<< DinLeft->getCpg()->getFrequency()*2.1/0.015<<" "<<DinRight->getCpg()->getFrequency()*2.1/0.015<< " " <<LeftLeg.at(0)<< " " <<RightLeg.at(0) <<" " << DinLeft->getCpg()->getFrequency()  << std::endl ;

		      // writing to motors either reflexive or cpg-based signals depending on enable
		      motors[0] = motorOutput[0]*!controllerEnable + LeftLeg.at(0)*controllerEnable ;//LHMusclTor;
			  motors[1] = motorOutput[1]*!controllerEnable + RightLeg.at(0)*controllerEnable ;//RHMusclTor;
			  motors[2] = motorOutput[2]*!controllerEnable + LeftLeg.at(1)*controllerEnable ;//LKMusclTor;
			  motors[3] = motorOutput[3]*!controllerEnable + RightLeg.at(1)*controllerEnable ;//RKMusclTor;
			  motors[4] = ubc+ubc_wabl;


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

}



void MuscleRunbotController::stepNoLearning(const sensor* sensors, int number_sensors, motor* motors, int number_motors) {
}

bool MuscleRunbotController::store(FILE* f) const {
}

bool MuscleRunbotController::restore(FILE* f) {
}

