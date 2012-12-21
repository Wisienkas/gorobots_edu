
/***************************************************************************
 *   Copyright (C) 2010 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                                                         *
 *   $Log: ashigaruController.cpp,v $                                         *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#include <selforg/controller_misc.h>
//#include <ode_robots/amosiisensormotordefinition.h>
#include "ashigaruController.h"

using namespace matrix;
using namespace std;
using namespace ASHIGARU;

// Circle Buff
// constructor
CirBuff::CirBuff(){
	maxBuff = 1000;
	buff.resize(maxBuff);
	currentBuffNum = 0; // 0 - (maxBuff - 1)
}

CirBuff::CirBuff(int _maxBuff){
	maxBuff = _maxBuff;
	buff.resize(maxBuff);
	currentBuffNum = 0; // 0 - (maxBuff - 1)
}

// destructor	]
CirBuff::~CirBuff(){
}

// addBuff
void CirBuff::addBuff(double value){
	++currentBuffNum;
	if(currentBuffNum >= maxBuff) currentBuffNum = currentBuffNum - maxBuff;
	buff.at(currentBuffNum) = value;
	return;
}

// getCurrentBuff
double CirBuff::getCurBuff() const{
	return buff.at(currentBuffNum);
}

// getPrvBuffer
double CirBuff::getPrvBuff(int prvCount) const{
	int buffNum = currentBuffNum;
	buffNum -= prvCount;
	if(buffNum < 0)buffNum += maxBuff;

	if(buffNum >= 0 && buffNum < maxBuff)return buff.at(buffNum);
	else return -1;
}



AshigaruController::AshigaruController(std::string name, const AshigaruConf& _aConf, const AshigaruControllerConf& _conf)
: AbstractController("AshigaruController", "$Id: ashigaruController.cpp,v 0.1 $"),obsMode(false), conf(_conf), robotConf(_aConf){
	// time step
	t = 0;
	// initialize
	for(int i = 0; i< 3; ++i){
	  // oscilation Set
	   // initial phase
	  osc[i].cpgPhase = conf.oscConf[i].initialPhase;
	   // other param init
	  osc[i].footBias = 0.;
	  osc[i].outPhase = 0.;
	  osc[i].count = 0;
	  osc[i].prvTouchF = false;
	}

  //file open
  std::string fileName;
  fileName = name;
  std::string a =  "Osc.txt";
  fileName += a;
  char fileName2[30];
  strcpy(fileName2,fileName.c_str());
  ofs.open(fileName2);
  std::cout << "make a file"<< std::endl;
  ofs << "# This is the Oscillation data of the neural bias and output and command data " << std::endl;
 // ofs << "# time(s),  outPutH1[1], outPutH2[1], finalOutT[1], finalOutC[1], biasH1[1], biasH2[1], footContact[1]"
 //		  ",  outPutH1[2], outPutH2[2], finalOutT[2], finalOutC[2], biasH1[2], biasH2[2], footContact[2]" << std::endl;
  ofs << "# 1 time(s), "
		  "2 osc[1].cpgPhase, 3 osc[1].outPhase, 4 -sin(osc[1].outPhase), "
		  "5 osc[1].footBias, 6 sDataSet.forceData[1], "
		  "7 ctr[1].l_toePos[x], 8 ctr[1].l_toePos[y], 9 ctr[1].l_toePos[z], "
		  "10 ctr[1].tAngle, 11 ctr[1].cAngle, 12 ctr[1].fAngle,           "

		  "13 osc[2].cpgPhase, 14 osc[2].outPhase, 15 -sin(osc[2].outPhase), "
		  "16 osc[2].footBias, 17 sDataSet.forceData[2], "
		  "18 ctr[2].l_toePos[x], 19 ctr[2].l_toePos[y], 20 ctr[2].l_toePos[z], "
		  "21 ctr[2].tAngle, 22 ctr[2].cAngle, 23 ctr[2].fAngle,           " << endl;

  //file open
  fileName = name;
  a =  "Param.txt";
  fileName += a;
  strcpy(fileName2,fileName.c_str());
  ofs2.open(fileName2);
  std::cout << "make a Parameter file"<< std::endl;
  ofs2 << "# This contains the Parameters of this simulation " << std::endl;
  ofs2 << " " << std::endl;

  //file open
  fileName = name;
  a =  "Obs.txt";
  fileName += a;
  strcpy(fileName2,fileName.c_str());
  ofs3.open(fileName2);
  std::cout << "make a Obs file"<< std::endl;
  ofs3 << "# This contains Observe Data of this simulation " << std::endl;
  ofs3 << "# time phase0 phase1 phase2 ... , diff 1-0 diff 2-1 .." << std::endl;


  // prepare name;
  Configurable::insertCVSInfo(name, "$RCSfile: AshigaruController.cpp,v $",
      "$Revision: 0.1 $");
}


AshigaruController::~AshigaruController() {
	ofs.close();
	ofs2.close();
	ofs3.close();

}

void AshigaruController::init(int sensornumber, int motornumber, RandGen* randGen)
{
  // For initialize controller, we can restrict something
  //just for test, so I don't restrict any more
  assert(motornumber>=0);
}

/// performs one step (includes learning). Calculates motor commands from sensor
/// inputs.
void AshigaruController::step(const sensor* x_, int number_sensors,
    motor* y_, int number_motors) {
  stepNoLearning(x_, number_sensors, y_, number_motors);
}

/// performs one step without learning. Calulates motor commands from sensor
/// inputs.
void AshigaruController::stepNoLearning(const sensor* x_, int number_sensors,
    motor* y_, int number_motors) {

  //############ Program for analyzing the effect of phase reset ###########################


	// ####################### store the data of sensor ####################################
	// store the joint and force data
	for(int s = 0; s< CPGNUM;s++){
		sDataSet.jData[s].tAngle = x_[ASHIGARU::T0_as + s];
		sDataSet.jData[s].cAngle = x_[ASHIGARU::C0_as + s];
		sDataSet.jData[s].fAngle = x_[ASHIGARU::F0_as + s];
		sDataSet.jData[s].tTorque = x_[ASHIGARU::T0_ts + s];
		sDataSet.jData[s].cTorque = x_[ASHIGARU::C0_ts + s];
		sDataSet.jData[s].fTorque = x_[ASHIGARU::F0_ts + s];
		sDataSet.forceData[s] = x_[ASHIGARU::L0_fs + s];
	}

	// another data vectors
	for(int i = 0; i< 3; i++){
		sDataSet.pose[i] = x_[ASHIGARU::POSE_r + i];
		sDataSet.angVel[i] = x_[ASHIGARU::W_x + i];
		sDataSet.g_roboPos[i] = x_[ASHIGARU::GPOS_Rx + i];
		sDataSet.g_roboSpd[i] = x_[ASHIGARU::GSPD_Rx + i];
		sDataSet.g_cogPos[i] = x_[ASHIGARU::GPOS_COGx + i];
		sDataSet.g_legToePos[0][i] = x_[ASHIGARU::GPOS_L0x + i];
		sDataSet.g_legToePos[1][i] = x_[ASHIGARU::GPOS_L1x + i];
		sDataSet.g_legToePos[2][i] = x_[ASHIGARU::GPOS_L2x + i];
	}


#ifdef DEBUG
	std::cout << "sensor setting" << std::endl;
#endif

	// ####################### Oscilation Phase ############################################

	// ######## calculatin of the Phase Reset and inhibition ###############
	  for(int s =0 ; s < CPGNUM; s++){
		  // detect  whether it touched or not
		  	bool touchF = (sDataSet.forceData[s] > 0.01);

		  //Phase Reset + Inhibition + force]
		  	 // Inhibition duration
			if(osc[s].count > 0){
				//osc[s].footBias = - osc[s].iCoef * sDataSet.forceData[s];
				osc[s].footBias = - osc[s].iCoef * 0.3;
				osc[s].count -= 1;
			}

			 // The moment that the Leg contacts to the ground first!!
			else if( touchF && !osc[s].prvTouchF){
				// calculate this situation
				double w = 2. * M_PI * conf.oscConf[s].freq;
				double ph = osc[s].cpgPhase + w * 1./conf.controlFreq;

				 //Phase Reset situation
				//if(ph > (2.* M_PI * conf.oscConf[s].dutyRate) ){
				if(ph > (2.* M_PI * conf.oscConf[s].dutyRate + M_PI/2.) ){// for debug
					osc[s].footBias = 0. - ph;
					//std::cout << " >> PhaseReset + rate:" << (ph - (2.* M_PI * conf.oscConf[s].dutyRate)) / ((2.* M_PI * (1. - conf.oscConf[s].dutyRate) )) <<  std::endl;
				}
				 //Phase Inhibitation situation
				else if(ph < (2.* M_PI * conf.oscConf[s].dutyRate)){
					osc[s].iCoef = conf.oscConf[s].inhibiCoef * ph;// inhibit rate changes according to the time
					//osc[s].footBias = - osc[s].iCoef * sDataSet.forceData[s];
					osc[s].footBias = - osc[s].iCoef * 0.3;
					osc[s].count = (int)( 1. / conf.oscConf[s].freq /4. * conf.controlFreq );// the duration that inhibit lasts for
					//std::cout << " << phaseInhibition + rate:" << ph / (2.* M_PI * conf.oscConf[s].dutyRate) << std::endl;
				}
				 // ELSE (there are no need to do phase reset or inhibit)
				else{
					osc[s].footBias = 0.;
					//std::cout << " ** nothing" << std::endl;
				}
			}

			 // ELSE
			else{
				osc[s].footBias = 0.;
			}

			// update prvTouch
			osc[s].prvTouchF = touchF;
	  }

	  //If you do not use footBias
	  if( !conf.useFootBias  || t < (conf.biasStartTime * conf.controlFreq) ){
		  for(int s =0 ; s < CPGNUM; s++){
			  osc[s].footBias = 0.;
		  }
	  }


#ifdef DEBUG
	std::cout << "calc phase reset" << std::endl;
#endif


	// ########### Calculation of CPG oscillation #################################################
	//  CPG
	  // calculate CPG output
		// we use  simple CPG to generate oscillation pattern
		//  We apply CPG to each Leg
		//! ! this is for straight 6 legged situation, I should make a program which is adaptive to other connection patterns
		//  ! In this point, I should think later/////

	// CPG 0
	double w = 2. * M_PI * conf.oscConf[0].freq;
	osc[0].cpgPhase = osc[0].cpgPhase + w * 1./conf.controlFreq + osc[0].footBias;
	checkPhase(osc[0].cpgPhase);

	// CPG 1
	w = 2. * M_PI * conf.oscConf[1].freq;
	osc[1].cpgPhase = osc[1].cpgPhase + w * 1./conf.controlFreq - conf.twoCpgCoef * sin( osc[1].cpgPhase - osc[2].cpgPhase - conf.twoCPGAngleDiff ) + osc[1].footBias;
	checkPhase(osc[1].cpgPhase);

	// CPG 2
	w = 2. * M_PI * conf.oscConf[2].freq;
	osc[2].cpgPhase = osc[2].cpgPhase + w * 1./conf.controlFreq - conf.twoCpgCoef * sin( osc[2].cpgPhase - osc[1].cpgPhase + conf.twoCPGAngleDiff ) + osc[2].footBias;
	checkPhase(osc[2].cpgPhase);

#ifdef DEBUG
	std::cout << "CPG calc" << std::endl;
#endif

	// ########### Post processing of CPG oscillation (change duty ratio)#################################################
	//Phase Change
	// by this one,  I change the duration of stance and swing phase.
	// changed phase is represented as outPhase
	// For detail explanation, please see some papers about it
	//
	for(int s = 0; s<CPGNUM; s++){
		// change the output signal
		if(osc[s].cpgPhase < 2. * M_PI * conf.oscConf[s].dutyRate){
			osc[s].outPhase = osc[s].cpgPhase / 2. / conf.oscConf[s].dutyRate;
		}else{
			osc[s].outPhase = (osc[s].cpgPhase - 2. * M_PI * conf.oscConf[s].dutyRate) / (2. * ( 1. -  conf.oscConf[s].dutyRate)) + M_PI;
		}
	}

#ifdef DEBUG
	std::cout << "Post processing" << std::endl;
#endif

	// ########### Trajectory making and inverse kinematics   #############################################################
	// calculate trajectory for each leg toe
	//  Moving trajectory is made by sin and cosin function because it is easy to be caluculated
	//
	for(int s = 0; s < CPGNUM; s++){
		if(osc[s].outPhase <= M_PI){// if stance phase
			// calc moving trajectory
			ctr[s].l_toePos = conf.trjConf[s].centerTrj - conf.trjConf[s].moveDirection * (conf.trjConf[s].walkLength / 2.) * cos(osc[s].outPhase);
			// calc z direction moving trajectory
			ctr[s].l_toePos[2] = ctr[s].l_toePos[2] - conf.trjConf[s].stanceHeight * sin(osc[s].outPhase);
		}
		else{ // swing phase
			// calc moving trajectory
			ctr[s].l_toePos = conf.trjConf[s].centerTrj - conf.trjConf[s].moveDirection * (conf.trjConf[s].walkLength / 2.) * cos(osc[s].outPhase);
			// calc z direction moving trajectory
			ctr[s].l_toePos[2] = ctr[s].l_toePos[2] - conf.trjConf[s].swingHeight * sin(osc[s].outPhase);
		}

	// calculate inverse kinematics
		ctr[s].isSuccessInvKinema =  clacInvKinematics(ctr[s].l_toePos, ctr[s].tAngle, ctr[s].cAngle, ctr[s].fAngle);
	}

#ifdef DEBUG
	std::cout << "inverse kinematics" << std::endl;
#endif


	// ########## Set the control command to the motor ####################################################################
	// Notice the positive direction of the joint (it was scaled -1 to 1)
	//  In initial state......
	//   TC joint : -z axis is positive direction of angular vector
	//   CT joint : -y axis is positive
	//   FT joint : -y axis is positive

	// stopped Leg ;; this should be automatically determined, but at that time I ignore it
	y_[T0_m] = 0.;//
	y_[C0_m] =  1.;
	y_[F0_m] =  - 1.;

	// moving leg 1
	y_[T1_m] =  - ctr[1].tAngle / (M_PI / 2.);
	y_[C1_m] =  - ctr[1].cAngle / (M_PI / 2.);
	y_[F1_m] =  (M_PI / 2. - ctr[1].fAngle) / (M_PI / 2.);

	// moving leg 2
	y_[T2_m] =  - ctr[2].tAngle / (M_PI / 2.);
	y_[C2_m] =  - ctr[2].cAngle / (M_PI / 2.);
	y_[F2_m] =  (M_PI / 2. - ctr[2].fAngle) / (M_PI / 2.);


#ifdef DEBUG
	// moving leg 2
	std::cout << "out T2 :" << - ctr[2].tAngle
			<< " out C2 :" << - ctr[2].cAngle
			<< " out F2 :" << M_PI / 2. - ctr[2].fAngle << std::endl;
#endif


#ifdef DEBUG
	std::cout << "command sended" << std::endl;
#endif

  // logging
  if(t > 50){
	  /*// logger for PSN, VPN system
	  ofs << (double)t /10. << ", " << outputH1[1] << "  " << outputH2[1] << "  " << t_output.at(1) << "  " << c_output_post.at(1) << "  "
	  	  << footBiasH1[1] << " " << footBiasH2[1] << " " << x_[ASHIGARU::L1_fs] << "  "
	  	  << outputH1[2] << "  " << outputH2[2] << "  " << t_output.at(2) << "  " << c_output_post.at(2) << "  "
	  	  << footBiasH1[2] << " " << footBiasH2[2] << " " << x_[ASHIGARU::L2_fs] << std::endl;
	  */

	  ofs << (double)t /20. << ", "
			  << osc[1].cpgPhase << "  " << osc[1].outPhase << "  " << -sin(osc[1].outPhase) << " "
	  	  	  << osc[1].footBias << " " <<  sDataSet.forceData[1] << " "
	  	  	  << ctr[1].l_toePos[0] << " " <<  ctr[1].l_toePos[1] << " " << ctr[1].l_toePos[2]<< " "
	  	  	  << ctr[1].tAngle << " " << ctr[1].cAngle << " " << ctr[1].fAngle << "       "

	  	  	  << osc[2].cpgPhase << "  " << osc[2].outPhase << "  " << -sin(osc[2].outPhase) << " "
			  << osc[2].footBias << " " <<  sDataSet.forceData[2] << " "
			  << ctr[2].l_toePos[0] << " " <<  ctr[2].l_toePos[1] << " " << ctr[2].l_toePos[2]<< " "
			  << ctr[2].tAngle << " " << ctr[2].cAngle << " " << ctr[2].fAngle << "       "

	  	  	  << std::endl;

	  if(obsMode){
		  if(!mObject.empty()){
			  ofs3 << (double)t /20. << ", " ;
			  vector<AshigaruController*>::iterator it = mObject.begin();
			  // output the each phase
			  while( it != mObject.end() )
			  {
				  double a = (*it)->eGetPhase(1);
				  ofs3 << a  << " " ;
				  ++it;
			  }

			  // output the phase difference
			  ofs3 <<  ",   " ;
			  double bVal = 0;
			  it = mObject.begin();
			  bVal = (*it)->eGetPhase(1);
			  ++it;

			  while( it != mObject.end() )
			  {
				  double a = (*it)->eGetPhase(1);

				  double c = a - bVal;
				  if(c > 2. * M_PI) c = c - 2. * M_PI;
				  else if(c < 0) c = c + 2. * M_PI;

				  ofs3 << c  << " " ;
				  bVal =  a;
				  ++it;
			  }

			  ofs3 << std::endl;
		  }
	  }
  }

  // Parameter logging --
  if(t == 1){
	  ofs2 << "Information of Controller " << endl;
	  ofs2 << " Type    : " << "Simplified CPG" << endl;
	  ofs2 << " CnctType: " << "Straight Connect" << endl;
	  ofs2 << " ControllFrequency(Hz): " << conf.controlFreq << endl;
	  ofs2 << " Bias start time(s): " << conf.biasStartTime << endl;
	  ofs2 << " TwoCPGcnctCoef: " << conf.twoCpgCoef << ", TwoCPGPhaseDiff: " << conf.twoCPGAngleDiff <<  endl;

	  ofs2 << "" << endl;
	  ofs2 << "Parameter for CPG " << endl;

	  ofs2 << "Elements           : " << "CPG0 value,  CPG1 Value,  CPG2 Value ############################" << endl;
	  ofs2 << " RobotFrequency(Hz): " << conf.oscConf[0].freq <<",  " << conf.oscConf[1].freq <<",  " << conf.oscConf[2].freq << endl;
	  ofs2 << " StanceRatio       : " << conf.oscConf[0].dutyRate << ",  " <<conf.oscConf[1].dutyRate << ",  " <<conf.oscConf[2].dutyRate <<  endl;
	  ofs2 << " initialPhase      : " << conf.oscConf[0].initialPhase << ",  "<< conf.oscConf[1].initialPhase << ",  " <<  conf.oscConf[2].initialPhase << endl;
	  ofs2 << " inhibiCoef        : " << conf.oscConf[0].inhibiCoef << ",  " << conf.oscConf[1].inhibiCoef << ",  " << conf.oscConf[2].inhibiCoef << endl;
	  ofs2 <<  endl;
	  ofs2 <<  endl;

	  ofs2 << "Parameter for Trajectory making " << endl;
	  ofs2 << " In this simulation CPG0 is ignored " << endl;
	  ofs2 << "Elements           : " << "CPG0 value,  CPG1 Value,  CPG2 Value ############################" << endl;
	  ofs2 << " CenterPos(m)      : " << "("<< conf.trjConf[0].centerTrj[0] <<"," << conf.trjConf[0].centerTrj[1] <<"," << conf.trjConf[0].centerTrj[2] <<")"
			  << "("<< conf.trjConf[1].centerTrj[0] <<"," << conf.trjConf[1].centerTrj[1] <<"," << conf.trjConf[1].centerTrj[2] <<")"
			  << "("<< conf.trjConf[2].centerTrj[0] <<"," << conf.trjConf[2].centerTrj[1] <<"," << conf.trjConf[2].centerTrj[2] <<")" << endl;
	  ofs2 << " Direction Vector  : " << "("<< conf.trjConf[0].moveDirection[0] <<"," << conf.trjConf[0].moveDirection[1] <<"," << conf.trjConf[0].moveDirection[2] <<")"
			  << "("<< conf.trjConf[1].moveDirection[0] <<"," << conf.trjConf[1].moveDirection[1] <<"," << conf.trjConf[1].moveDirection[2] <<")"
			  << "("<< conf.trjConf[2].moveDirection[0] <<"," << conf.trjConf[2].moveDirection[1] <<"," << conf.trjConf[2].moveDirection[2] <<")" << endl;
	  ofs2 << " Swing Height      : " << conf.trjConf[0].swingHeight <<",  " << conf.trjConf[1].swingHeight <<",  " << conf.trjConf[2].swingHeight  << endl;
	  ofs2 << " Stance Height     : " << conf.trjConf[0].stanceHeight <<",  " << conf.trjConf[1].stanceHeight <<",  " << conf.trjConf[2].stanceHeight  << endl;
	  ofs2 << " walk Length       : " << conf.trjConf[0].walkLength <<",  " << conf.trjConf[1].walkLength <<",  " << conf.trjConf[2].walkLength  << endl;

  }

  // update step counter
  t++;

#ifdef DEBUG
	std::cout << "1step has finished , count " << t << std::endl;
#endif


}

/** stores the controller values to a given file. */
bool AshigaruController::store(FILE* f) const {
  return true;
}

/** loads the controller values from a given file. */
bool AshigaruController::restore(FILE* f) {
  return true;
}

/** get default configulation **/
AshigaruControllerConf AshigaruController::getDefaultConf() {
  AshigaruControllerConf c;

  // control Frequency
  c.controlFreq = 20.;

  // cnct param between two CPGs
  c.twoCpgCoef = 0.05;
  // diference in angle between two CPGs
  c.twoCPGAngleDiff = M_PI;

  // use Foot bias syustem or not
  c.useFootBias = true;
  // start time (s)
  c.biasStartTime = 7.5;

  // configurate osc and trj param
  for(int i =0;i<3;i++){
	  // for oscilation
	  c.oscConf[i].freq = 0.2;
	  c.oscConf[i].dutyRate = 0.5;
	  c.oscConf[i].inhibiCoef = 0.1;
	  c.oscConf[i].initialPhase = 0.;

	  // for traj
	  c.trjConf[i].centerTrj = osg::Vec3d(0.15, 0., -0.1);
	  c.trjConf[i].moveDirection = osg::Vec3d(0., 1., 0.);
	  c.trjConf[i].walkLength = 0.2;
	  c.trjConf[i].stanceHeight = 0.;
	  c.trjConf[i].swingHeight = 0.02;
  }

  // configure cnct param
   // is connected ??
  c.isBodyCncted[0] = 0;
  c.isBodyCncted[1] = -1;
  c.isBodyCncted[2] = -1;
   // does it connect to others
  c.isLegUsedCnct[0] = 0;
  c.isLegUsedCnct[1] = -1;
  c.isLegUsedCnct[2] = -1;

  return c;
}

/** get straight configulation **/
AshigaruControllerConf AshigaruController::getStraightCaseConf(){
  AshigaruControllerConf c;

  // control Frequency
  c.controlFreq = 20.;

  // cnct param between two CPGs
  c.twoCpgCoef = 0.05;
  // diference in angle between two CPGs
  c.twoCPGAngleDiff = M_PI;

  // use Foot bias syustem or not
  c.useFootBias = true;
  // start time (s)
  c.biasStartTime = 20;

  // configurate osc and trj param
  for(int i =0;i<3;i++){
	  // for oscilation
	  c.oscConf[i].freq = 0.2;//(Hz)
	  c.oscConf[i].dutyRate = 0.8;
	  c.oscConf[i].inhibiCoef = 0.1;//0.1;
	  c.oscConf[i].initialPhase = 0.;
  }
  c.oscConf[1].initialPhase = 0.;
  c.oscConf[2].initialPhase = M_PI;

  // for traj
  // anything is OK for 0
   c.trjConf[0].centerTrj = osg::Vec3d(0.15, 0., -0.1);
   c.trjConf[0].moveDirection = osg::Vec3d(0., 1., 0.);
   c.trjConf[0].walkLength = 0.;
   c.trjConf[0].stanceHeight = 0.;
   c.trjConf[0].swingHeight = 0.;

  // about 1
   // length from a base ]
   double l = 0.15; // direction that is vertical to moving direction
   double lx = -0.08;//- 0.08;//-0.05;// moving direction
   double h = - 0.12; // height of robot

   c.trjConf[1].centerTrj = osg::Vec3d( l * cos(M_PI/6.) + lx * sin(M_PI/6.), -l * sin(M_PI/6.) + lx * cos(M_PI/6.) , h);
   c.trjConf[1].moveDirection = osg::Vec3d( -cos(M_PI/3.), -sin(M_PI/3.), 0.);
   c.trjConf[1].walkLength = 0.1;//0.1;
   c.trjConf[1].stanceHeight = 0.0;//0.01;
   c.trjConf[1].swingHeight = 0.06;//0.04;

   c.trjConf[2].centerTrj = osg::Vec3d( l * cos(M_PI/6.)+ lx * sin(M_PI/6.), l * sin(M_PI/6.) - lx * cos(M_PI/6.), h);
   c.trjConf[2].moveDirection = osg::Vec3d( -sin(M_PI/6.), cos(M_PI/6.), 0.);
   c.trjConf[2].walkLength = 0.1;//0.1;
   c.trjConf[2].stanceHeight = 0.0;//0.01;
   c.trjConf[2].swingHeight = 0.06;//0.04;

  // configure cnct param
   // is connected ??
  c.isBodyCncted[0] = 0;
  c.isBodyCncted[1] = -1;
  c.isBodyCncted[2] = -1;
   // does it connect to others
  c.isLegUsedCnct[0] = 0;
  c.isLegUsedCnct[1] = -1;
  c.isLegUsedCnct[2] = -1;

  return c;
}


// This is the eFunction to analyze this oscillation from external viewer
// For Tempolary investigation
double AshigaruController::eGetPhase(int ch){
	if(ch >= 0 && ch < 3){
		return osc[ch].cpgPhase;
	}else return -1;
}

// get oscillation data
AshigaruController::oscSet AshigaruController::eGetOscSet(int ch){
	if(ch >= 0 && ch < 3){
		return osc[ch];
	}else return osc[1];
}

// get control data
AshigaruController::ctrSet AshigaruController::eGetCtrSet(int ch){
	if(ch >= 0 && ch < 3){
		return ctr[ch];
	}else return ctr[1];
}

// get sensed data
AshigaruController::sensedDataSet AshigaruController::eGetSensedData(void){
	return sDataSet;
}

// get configuration of Ashigaru controller
AshigaruControllerConf AshigaruController::eGetAshigaruCtrConf(void){
	return conf;
}

// For Phase Diff investigation
bool AshigaruController::setObserverMode(){
	obsMode = true;
	return true;
}

// tell the address of this controller
bool AshigaruController::addObject(AshigaruController* obj){
	if(obsMode){
		mObject.push_back(obj);
		return true;
	}
	else return false;
}

// check the phase
void AshigaruController::checkPhase(double& phase){
	if(phase >= 2. * M_PI) phase = phase - 2. * M_PI;
	else if(phase < 0) phase = phase + 2. * M_PI;
}

// inv Kinematics
bool AshigaruController::clacInvKinematics(osg::Vec3d toePos, double& tAng, double& cAng, double& fAng){
	// In this function we can get two solution but I take anly one solution that is more optimal for robot
	// It means I take only the solution that make leg convex shape

	// new param
	double x = toePos[0];
	double y = toePos[1];
	double z = toePos[2];

	double l0 = robotConf.jLength.length_TCJ_to_CTJ;
	double l1 = robotConf.jLength.length_CTJ_to_FTJ;
	double l2 = robotConf.jLength.length_FTJ_to_Toe;

	double t = sqrt(x*x + y*y) - l0;
	double p = sqrt(t*t + z*z);

	// If the system is singular configuration, calc is over
	if( fabs((l1*l1 + p*p - l2*l2) / (2.*l1*p)) > 1. || fabs((l1*l1 + l2*l2 - p*p)/(2.*l1*p)) > 1. ){
		std::cout << "@@ fail to calc inv kinematics!! change trajectry!!" << std::endl;
		return false;
	}

	// calculate inv Kinematics
	tAng = atan2(y, x);
	cAng = atan2(-z,t) - acos((l1*l1 + p*p - l2*l2) / (2.*l1*p) );
	fAng = M_PI - acos((l1*l1 + l2*l2 - p*p)/(2.*l1*p));

	return true;
}


