
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
 *   $Log: hexabotController.cpp,v $                                         *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#include <selforg/controller_misc.h>
//#include <ode_robots/amosiisensormotordefinition.h>
#include "hexabotController.h"
#define CHECK_PHASE_RESET
#define DEBUG2

using namespace matrix;
using namespace std;
using namespace HEXABOT;


// Circle Buff
// constructor
CirBuff::CirBuff(){
    maxBuff = 60000;
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



HexabotController::HexabotController(std::string name, HexabotConf& _aConf, HexabotControllerConf& _conf, bool _fileFlag)
    : AbstractController("HexabotController", "$Id: hexabotController.cpp,v 0.1 $"),obsMode(false), conf(_conf), robotConf(_aConf), fileFlag(_fileFlag)
{
	// time step
	t = 0;
	// initialize
    for(int i = 0; i< CPGNUM; ++i){
	  // oscilation Set
	   // initial phase
	  osc[i].cpgPhase = conf.oscConf[i].initialPhase;
      osc[i].filtPhase = conf.oscConf[i].initialPhase;
      osc[i].touchDownPhase = 0.;
	   // other param init
	  osc[i].footBias = 0.;
	  osc[i].outPhase = 0.;
      osc[i].count = 0;
	  osc[i].prvTouchF = false;
      osc[i].prvTouchTime = 0.;
      osc[i].iCoef = 0.;
      phaseResetFlag[i] = true;
	}

  //file open
	if(fileFlag){
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
				  "10 ctr[1].tAngle, 11 ctr[1].cAngle, 12 ctr[1].fAngle,"
				  "13 sensed[1].tAngle, 14 sensed[1].cAngle, 15 sensed[1].fAngle,               "

				  "16 osc[2].cpgPhase, 17 osc[2].outPhase, 18 -sin(osc[2].outPhase), "
				  "19 osc[2].footBias, 20 sDataSet.forceData[2], "
				  "21 ctr[2].l_toePos[x], 22 ctr[2].l_toePos[y], 23 ctr[2].l_toePos[z], "
				  "24 ctr[2].tAngle, 25 ctr[2].cAngle, 26 ctr[2].fAngle,"
				  "27 sensed[2].tAngle, 28 sensed[2].cAngle, 29 sensed[2].fAngle,               "<< endl;

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
	}

    // file open
    ofsP.open("//home//ambe//Dropbox//AshigaruShare//hexData//amos//realExp//phaseReset.dat");
    if( ofsP.is_open() ){
        ofsP << "# This is the file to output every phase resetting situation (time is different from the simulation time)" << std::endl;
        ofsP << "# 1 time, 2 LegNum (1-6), 3 Phase of touchDown, 4 roll, 5 pitch " << std::endl;
    }
	// prepare name;
    //Configurable::insertCVSInfo(name, "$RCSfile: HexabotController.cpp,v $",
    //  "$Revision: 0.1 $");
}


HexabotController::~HexabotController() {
	if(fileFlag){
		ofs.close();
		ofs2.close();
		ofs3.close();
	}

    if(ofsP.is_open()) ofsP.close();
}

void HexabotController::init(int sensornumber, int motornumber, RandGen* randGen)
{
  // For initialize controller, we can restrict something
  //just for test, so I don't restrict any more
  assert(motornumber>=0);
}

/// performs one step (includes learning). Calculates motor commands from sensor
/// inputs.
void HexabotController::step(const sensor* x_, int number_sensors,
    motor* y_, int number_motors) {
  HexabotController::stepNoLearning(x_, number_sensors, y_, number_motors);
}

/// performs one step without learning. Calulates motor commands from sensor
/// inputs.
void HexabotController::stepNoLearning(const sensor* x_, int number_sensors,
    motor* y_, int number_motors) {
    static double time = 0;

  //############ Program for analyzing the effect of phase reset ###########################

	// ####################### store the data of sensor ####################################
	// store the joint and force data
	for(int s = 0; s< CPGNUM;s++){
        sDataSet.jData[s].tAngle = - x_[HEXABOT::T1_as + s] * (M_PI / 2.);
        sDataSet.jData[s].cAngle = - x_[HEXABOT::C1_as + s] * (M_PI / 2.);
        sDataSet.jData[s].fAngle = - (x_[HEXABOT::F1_as + s] * (M_PI / 2.) - (M_PI / 2.));
        sDataSet.jData[s].tTorque = x_[HEXABOT::T1_ts + s];
        sDataSet.jData[s].cTorque = x_[HEXABOT::C1_ts + s];
        sDataSet.jData[s].fTorque = x_[HEXABOT::F1_ts + s];
        sDataSet.forceData[s] = x_[HEXABOT::L1_fs + s];
	}

	// another data vectors
    for(int i = 0; i< 3; i++){
        sDataSet.pose[i] = x_[HEXABOT::POSE_r + i];
        sDataSet.angVel[i] = x_[HEXABOT::W_x + i];
        sDataSet.g_roboPos[i] = x_[HEXABOT::GPOS_Rx + i];
        sDataSet.g_roboSpd[i] = x_[HEXABOT::GSPD_Rx + i];
        sDataSet.g_cogPos[i] = x_[HEXABOT::GPOS_COGx + i];
        sDataSet.g_legToePos[0][i] = x_[HEXABOT::GPOS_L1x + i];
        sDataSet.g_legToePos[1][i] = x_[HEXABOT::GPOS_L2x + i];
        sDataSet.g_legToePos[2][i] = x_[HEXABOT::GPOS_L3x + i];
        sDataSet.g_legToePos[3][i] = x_[HEXABOT::GPOS_L4x + i];
        sDataSet.g_legToePos[4][i] = x_[HEXABOT::GPOS_L5x + i];
        sDataSet.g_legToePos[5][i] = x_[HEXABOT::GPOS_L6x + i];
    }


#ifdef DEBUG
	std::cout << "sensor setting" << std::endl;
#endif

	// ####################### Oscilation Phase ############################################

    // control dt
    double dt = 1./conf.controlFreq;
    // phase resetFlag
    bool pResetFlag[CPGNUM] = {false,false,false,false,false,false};
    // update time
    time = time + dt;

    static double prvResetTime[6] = {0.};
    static double prvSTime[6] = {0.};
    static int touchCount[6] = {0};
    static double prvResetPh[6] = {6.28, 6.28, 6.28, 6.28, 6.28, 6.28};

    // ######## calculatin of the Phase Reset and inhibition ###############
	  for(int s =0 ; s < CPGNUM; s++){
		  // detect  whether it touched or not
          if(eIsLegContact(s)){
              touchCount[s]++;
          }else{
              touchCount[s] = 0;
          }
          bool touchF = (touchCount[s] > 2); //(sDataSet.forceData[s] > 0.01);

          // What they do about phase is different from PC_MODE
            //Phase Reset + Inhibition +
            if(conf.PC_MODE == PC_PR_IN){
                 // Inhibition duration
                if(osc[s].count > 0){
                    //osc[s].footBias = - osc[s].iCoef * sDataSet.forceData[s];
                    osc[s].footBias = - osc[s].iCoef;//
                    osc[s].count -= 1;//
                }
                 // The moment that the Leg contacts to the ground first!!
                else if( touchF && !osc[s].prvTouchF){
                    // calculate this situation
                    double w = 2. * M_PI * conf.oscConf[s].freq;
                    double ph = osc[s].cpgPhase + w * dt;
                    osc[s].touchDownPhase = ph;

                     //Phase Reset situation
                    //if(ph > (2.* M_PI * conf.oscConf[s].dutyRate) ){
                    if(ph > (2.* M_PI * conf.oscConf[s].dutyRate + (2.* M_PI * (1. - conf.oscConf[s].dutyRate))/2.) ){// for debug
                        osc[s].footBias = 0. - ph / dt;
                        pResetFlag[s] = true;
    #ifdef CHECK_PHASE_RESET
                        std::cout << " >> PhaseReset + rate:" << (ph - (2.* M_PI * conf.oscConf[s].dutyRate)) / ((2.* M_PI * (1. - conf.oscConf[s].dutyRate) )) <<  std::endl;
    #endif
                    }
                     //Phase Inhibitation situation
                    else if(ph < (2.* M_PI * conf.oscConf[s].dutyRate )){// test
                        osc[s].iCoef = conf.oscConf[s].inhibiCoef * ph;// inhibit rate changes according to the time
                        // //osc[s].footBias = - osc[s].iCoef * sDataSet.forceData[s];
                        osc[s].footBias = - osc[s].iCoef;
                        osc[s].count = (int)( 1. / conf.oscConf[s].freq /6. * conf.controlFreq );// the duration that inhibit lasts for
    #ifdef CHECK_PHASE_RESET
                        std::cout << " << phaseInhibition + rate:" << ph / (2.* M_PI * conf.oscConf[s].dutyRate) << " ph:" << ph<< std::endl;
    #endif
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
            }
            // PhaseReset only on swing phase
            else if(conf.PC_MODE == PC_PR){
               // The moment that the Leg contacts to the ground first!!
               if( touchF && !osc[s].prvTouchF){
                   // calculate this situation
                   double w = 2. * M_PI * conf.oscConf[s].freq;
                   double ph = osc[s].cpgPhase + w * dt;
                   osc[s].touchDownPhase = ph;

                    //Phase Reset situation
                   //if(ph > (2.* M_PI * conf.oscConf[s].dutyRate) ){
                   if(ph > (2.* M_PI * conf.oscConf[s].dutyRate + (2.* M_PI * (1. - conf.oscConf[s].dutyRate))/2.)){// for debug
                       osc[s].footBias = 0. - ph / dt;
                        pResetFlag[s] = true;
#ifdef CHECK_PHASE_RESET
                        std::cout << " >> PhaseReset + rate:" << (ph - (2.* M_PI * conf.oscConf[s].dutyRate)) / ((2.* M_PI * (1. - conf.oscConf[s].dutyRate) )) <<  std::endl;
#endif
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

            }
            // PhaseReset only in both phase
            else if(conf.PC_MODE == PC_PR_PR){
               // The moment that the Leg contacts to the ground first!!
               if( touchF && !osc[s].prvTouchF){
                   // calculate this situation
                   double w = 2. * M_PI * conf.oscConf[s].freq;
                   double ph = osc[s].cpgPhase + w * dt;
                   osc[s].touchDownPhase = ph;

                    //Phase Reset situation
                   if( ph > (2.* M_PI * conf.oscConf[s].dutyRate + (2.* M_PI * (1. - conf.oscConf[s].dutyRate))/2.) ){
                       osc[s].footBias = 2. * M_PI - ph;
                       pResetFlag[s] = true;
#ifdef CHECK_PHASE_RESET
                       std::cout << " >> PhaseReset + phaseRate (in swing ):" << ph / (2. * M_PI) <<  std::endl;
#endif
                   }
                   else if( ph < (2.* M_PI * conf.oscConf[s].dutyRate) ){
                       osc[s].footBias = 0. - ph / dt;
                        pResetFlag[s] = true;
#ifdef CHECK_PHASE_RESET
                       std::cout << " >> PhaseReset - phaseRate (in stance):" << ph / (2. * M_PI) <<  std::endl;
#endif
                   }
                   // ELSE (there are no need to do phase reset)
                   else{
                       osc[s].footBias = 0.;
                       //std::cout << " ** nothing" << std::endl;
#ifdef CHECK_PHASE_RESET
                       std::cout << " >> without PhaseReset Just!!:" << ph / (2. * M_PI) <<  std::endl;
#endif
                   }
               }
                // ELSE
               else{
                   osc[s].footBias = 0.;
               }

            }
            // Inhibi in both phase
            else if(conf.PC_MODE == PC_IN_IN){
                // Inhibition duration
                if(osc[s].count > 0){
                    //osc[s].footBias = - osc[s].iCoef * sDataSet.forceData[s];
                    osc[s].footBias = - osc[s].iCoef;//
                    osc[s].count -= 1;//
                }

                // The moment that the Leg contacts to the ground first!!
                else if( touchF && !osc[s].prvTouchF){
                    // calculate this situation
                    double w = 2. * M_PI * conf.oscConf[s].freq;
                    double ph = osc[s].cpgPhase + w * dt;
                    osc[s].touchDownPhase = ph;

                    //Phase Reset situation
                    //if(ph > (2.* M_PI * conf.oscConf[s].dutyRate) ){
                    if(ph > (2.* M_PI * conf.oscConf[s].dutyRate + (2.* M_PI * (1. - conf.oscConf[s].dutyRate))/2.) ){// for debug
                        osc[s].iCoef = conf.oscConf[s].inhibiCoef * (ph - 2.*M_PI );// inhibit rate changes according to the time
                        // //osc[s].footBias = - osc[s].iCoef * sDataSet.forceData[s];
                        osc[s].footBias = - osc[s].iCoef;
                        osc[s].count = (int)( 1. / conf.oscConf[s].freq /6. * conf.controlFreq );// the duration that inhibit lasts for
                        #ifdef CHECK_PHASE_RESET
                        std::cout << " << phaseInhibition(ResetSitu) leg:" << s+1 << " + rate:" << ph / (2.* M_PI * conf.oscConf[s].dutyRate) << " ph:" << ph<< std::endl;
                        #endif
                    }
                    //Phase Inhibitation situation
                    else if(ph < (2.* M_PI * conf.oscConf[s].dutyRate )){// test
                        osc[s].iCoef = conf.oscConf[s].inhibiCoef * ph;// inhibit rate changes according to the time
                        // //osc[s].footBias = - osc[s].iCoef * sDataSet.forceData[s];
                        osc[s].footBias = - osc[s].iCoef;
                        osc[s].count = (int)( 1. / conf.oscConf[s].freq /6. * conf.controlFreq );// the duration that inhibit lasts for
                        #ifdef CHECK_PHASE_RESET
                        std::cout << " << phaseInhibition leg:" << s+1<< " + rate:" << ph / (2.* M_PI * conf.oscConf[s].dutyRate) << " ph:" << ph<< std::endl;
                        #endif
                    }

                    // ELSE (there are no need to do phase reset or inhibit)
                    else{
                        osc[s].footBias = 0.;
                        //std::cout << " ** nothing" << std::endl;
                    }
                    // ELSE
                }else{
                    osc[s].footBias = 0.;
                }
            }
            // Phase reset by low pass
            else if(conf.PC_MODE == PC_PR_LPASS){
                double e = 2.71828182846;

                // if the leg already contact the ground when the phase reset starts, we reset of course
                double ph2 = osc[s].cpgPhase + 2. * M_PI * conf.oscConf[s].freq * dt;
                if ( ph2 < 2.* M_PI  && ph2 > (2.* M_PI * conf.oscConf[s].dutyRate + (2.* M_PI * (1. - conf.oscConf[s].dutyRate)) * 3./4.) && (time - prvSTime[s]) > 7.){
                    prvSTime[s] =  time;
                    if(touchF){ osc[s].prvTouchF = false; }
                }

                // The moment that the Leg contacts to the ground first!!
                if(s == 5 || s == 1 || s == 0){
                    if( touchF && !osc[s].prvTouchF){
                    // calculate this situation
                    double w = 2. * M_PI * conf.oscConf[s].freq;
                    double ph = osc[s].cpgPhase + w * dt;
                    osc[s].touchDownPhase = ph;

                    // update prvTouch time
                    osc[s].prvTouchTime = time;

                    // touch in swing phase)
                    //if(ph > (2.* M_PI * conf.oscConf[s].dutyRate) ){
                    // TO deal chattering, I change a little bit
                    // Also because of the noise.. some peaks occurs
                    if(ph > (2.* M_PI * conf.oscConf[s].dutyRate + (2.* M_PI * (1. - conf.oscConf[s].dutyRate)) * 3./4.)  && time > (prvResetTime[s]+5.)  ){// for debug
//                        if( fabs(prvResetPh[s] - ph) > 0.3){
//                            ph = prvResetPh[s];
//                            std::cout << "noise >>>>>>>>.." << std::endl;
//                        }
//                        else {prvResetPh[s] = ph;}
                        osc[s].iCoef = conf.oscConf[s].inhibiCoef * ( 2.*M_PI - ph );
                        #ifdef CHECK_PHASE_RESET
                        std::cout << " << phaseInhibition(ResetSitu) leg:" << s+1 << " + rate:" << ph / (2.* M_PI * conf.oscConf[s].dutyRate) << " ph:" << ph<< std::endl;
                        #endif
                        ofsP << time << "  " << s + 1 << "  " << ph << " " << eGetSensedData().pose.x() << " " << eGetSensedData().pose.y() << std::endl;
                        prvResetTime[s] = time;
                    }
                    // touch in stance phase)
                    else if(0){//ph < (2.* M_PI * conf.oscConf[s].dutyRate )){// test
                        osc[s].iCoef = conf.oscConf[s].inhibiCoef * ( - ph );
                        #ifdef CHECK_PHASE_RESET
                        std::cout << " << phaseInhibition leg:" << s+1<< " + rate:" << ph / (2.* M_PI * conf.oscConf[s].dutyRate) << " ph:" << ph<< std::endl;
                        #endif
                    }
                    // ELSE (there are no need to do phase reset or inhibit)
                    else{
                        //osc[s].iCoef = 0;
                        std::cout << "something unhappy happens" << std::endl;
                    }
                }
                }

                // calculate input.....
                osc[s].footBias = (1./conf.TFactor) * osc[s].iCoef * pow(e, - ( time - osc[s].prvTouchTime) /conf.TFactor );
                //std::cout << " footBias:" << osc[s].iCoef <<", " << pow(e, - ( time - osc[s].prvTouchTime) /conf.TFactor ) << ", " <<osc[s].footBias  << std::endl;

            }
            // without Period Changing Mechanism
            else if(conf.PC_MODE == PC_NONE){
                // do nothing
                osc[s].footBias = 0.;
            }

          // Finalize thing
			// update prvTouch
			osc[s].prvTouchF = touchF;
	  }

      //Without PC method until start bias time
      if(conf.PC_MODE == PC_NONE  || t < (conf.biasStartTime * conf.controlFreq) ){
          for(int s =0 ; s < CPGNUM; s++){
			  osc[s].footBias = 0.;
              pResetFlag[s] = false;
		  }
       }

      // check phase reset flag for each CPG
      for(int s =0 ; s < CPGNUM; s++){
          if(!phaseResetFlag[s]){
            osc[s].footBias = 0.;
            pResetFlag[s] = false;
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
        //
        //           cpg 3  |  cpg 0
        //           cpg 4  |  cpg 1
        //           cpg 5  |  cpg 2
        //

    // Runge kutta
    double k1[2],k2[2],k3[2],k4[2];

    // CPGs are calcukated by RungeKutta

    for (int i=0;i<3;i++){
        // CPG 1,2 w
        double w1 = 2. * M_PI * conf.oscConf[i].freq;
        double w2 = 2. * M_PI * conf.oscConf[i+3].freq;

        // One side TEST phase
        if(conf.is_oneSidePhaseModulation){
            osc[i+3].footBias = 0.;
        }

        // RungeKutta
        k1[0] = (w1 - conf.twoCpgCoef * sin( osc[i].cpgPhase - osc[i+3].cpgPhase - conf.twoCPGAngleDiff ) + osc[i].footBias);
        k1[1] = (w2  - conf.twoCpgCoef * sin( osc[i+3].cpgPhase - osc[i].cpgPhase + conf.twoCPGAngleDiff ) + osc[i+3].footBias);

        k2[0] = (w1 - conf.twoCpgCoef * sin( (osc[i].cpgPhase + k1[0]*dt/2.) - (osc[i+3].cpgPhase + k1[1]*dt/2.) - conf.twoCPGAngleDiff ) + osc[i].footBias);
        k2[1] = (w2  - conf.twoCpgCoef * sin( (osc[i+3].cpgPhase + k1[1]*dt/2.) - (osc[i].cpgPhase + k1[0]*dt/2.) + conf.twoCPGAngleDiff ) + osc[i+3].footBias);

        k3[0] = (w1 - conf.twoCpgCoef * sin( (osc[i].cpgPhase + k2[0]*dt/2.) - (osc[i+3].cpgPhase + k2[1]*dt/2.) - conf.twoCPGAngleDiff ) + osc[i].footBias);
        k3[1] = (w2  - conf.twoCpgCoef * sin( (osc[i+3].cpgPhase + k2[1]*dt/2.) - (osc[i].cpgPhase + k2[0]*dt/2.) + conf.twoCPGAngleDiff ) + osc[i+3].footBias);

        k4[0] = (w1 - conf.twoCpgCoef * sin( (osc[i].cpgPhase + k3[0]*dt) - (osc[i+3].cpgPhase + k3[1]*dt) - conf.twoCPGAngleDiff ) + osc[i].footBias);
        k4[1] = (w2  - conf.twoCpgCoef * sin( (osc[i+3].cpgPhase + k3[1]*dt) - (osc[i].cpgPhase + k3[0]*dt) + conf.twoCPGAngleDiff ) + osc[i+3].footBias);

        osc[i].cpgPhase = osc[i].cpgPhase + (k1[0] + k2[0] * 2. + k3[0] * 2. + k4[0]) * (dt/6.);
        osc[i+3].cpgPhase = osc[i+3].cpgPhase + (k1[1] + k2[1] * 2. + k3[1] * 2. + k4[1]) * (dt/6.);
    }

    // if the phase reset is applied
    for(int i=0;i<CPGNUM;i++){
        if(pResetFlag[i]){
            osc[i].cpgPhase = 0.;
            pResetFlag[i] = false;
        }
    }

    /*
    //CPG 1
    osc[1].cpgPhase = osc[1].cpgPhase + (w1 - conf.twoCpgCoef * sin( osc[1].cpgPhase - osc[2].cpgPhase - conf.twoCPGAngleDiff ) + osc[1].footBias) * dt;
	// CPG 2
    osc[2].cpgPhase = osc[2].cpgPhase + (w2  - conf.twoCpgCoef * sin( osc[2].cpgPhase - osc[1].cpgPhase + conf.twoCPGAngleDiff ) + osc[2].footBias) * dt;
    */

#ifdef DEBUG
	std::cout << "CPG calc" << std::endl;
#endif
    // ########## Post processing of CPG oscillation by using 1 demensional delayed system ################################
    //  When We use phase reset also about inhibition, we apply the 1 demensional delayed system to make the change in phase gradual
    //
    if(conf.PC_MODE == PC_PR_PR){
        for(int s = 0;s<CPGNUM; s++){
            // calc T (time factor)
            // THis system will follow desired signal around 3*T
            double tFactor = 1./ conf.oscConf[s].freq * conf.TFactor;
            // control time dt
            double dt = 1. / conf.controlFreq;
            // calc Filt phase -- 1 demensional delayed system
            //   I assume that the phase diff is less than M_PI/2!! That means tFactor is less than 0.25
            //   This assumption is not so bad I think
            if(osc[s].cpgPhase < M_PI / 2. &&  osc[s].filtPhase > 3. /2. * M_PI){
                osc[s].filtPhase = osc[s].filtPhase + dt / tFactor * (osc[s].cpgPhase + 2. * M_PI - osc[s].filtPhase);
            }else if(osc[s].filtPhase < M_PI / 2. &&  osc[s].cpgPhase > 3. /2. * M_PI){
                osc[s].filtPhase = osc[s].filtPhase + dt / tFactor * (osc[s].cpgPhase  - osc[s].filtPhase - 2. * M_PI);
            }else{
                osc[s].filtPhase = osc[s].filtPhase + dt / tFactor * (osc[s].cpgPhase - osc[s].filtPhase);
            }

            // check phase
            checkPhase(osc[s].cpgPhase);
            checkPhase(osc[s].filtPhase);
        }
    }else{
        for(int s = 0;s<CPGNUM; s++){
            checkPhase(osc[s].cpgPhase);
            osc[s].filtPhase = osc[s].cpgPhase;
        }
    }

	// ########### Post processing of CPG oscillation (change duty ratio)#################################################
	//Phase Change
	// by this one,  I change the duration of stance and swing phase.
	// changed phase is represented as outPhase
	// For detail explanation, please see some papers about it
	//
	for(int s = 0; s<CPGNUM; s++){
		// change the output signal
        if(osc[s].filtPhase < 2. * M_PI * conf.oscConf[s].dutyRate){
            osc[s].outPhase = osc[s].filtPhase / 2. / conf.oscConf[s].dutyRate;
		}else{
            osc[s].outPhase = (osc[s].filtPhase - 2. * M_PI * conf.oscConf[s].dutyRate) / (2. * ( 1. -  conf.oscConf[s].dutyRate)) + M_PI;
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
             // move in constant speed.
            ctr[s].l_toePos = conf.trjConf[s].centerTrj - conf.trjConf[s].moveDirection * (conf.trjConf[s].walkLength / 2.) + conf.trjConf[s].moveDirection * conf.trjConf[s].walkLength * (osc[s].outPhase / M_PI);
             // move in cos factor
            // ctr[s].l_toePos = conf.trjConf[s].centerTrj - conf.trjConf[s].moveDirection * (conf.trjConf[s].walkLength / 2.) * cos(osc[s].outPhase);

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

    for(int i =0;i<CPGNUM;i++){
        // moving leg 1
        y_[T1_m + i] =  - ctr[i].tAngle / (M_PI / 2.);
        y_[C1_m + i] =  - ctr[i].cAngle / (M_PI / 2.);
        y_[F1_m + i] =  (M_PI / 2. - ctr[i].fAngle) / (M_PI / 2.);
    }

    /*
    // check disabled legs
	for(int i = 0; i< 3;i++){
		if(conf.disLeg[i]){
			y_[T0_m + i] =  0.;//
			y_[C0_m + i] =  1.;
			y_[F0_m + i] = -0.;
		}
	}
    */


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
	  if(fileFlag){
		  /*// logger for PSN, VPN system
		  ofs << (double)t /10. << ", " << outputH1[1] << "  " << outputH2[1] << "  " << t_output.at(1) << "  " << c_output_post.at(1) << "  "
              << footBiasH1[1] << " " << footBiasH2[1] << " " << x_[HEXABOT::L1_fs] << "  "
			  << outputH1[2] << "  " << outputH2[2] << "  " << t_output.at(2) << "  " << c_output_post.at(2) << "  "
              << footBiasH1[2] << " " << footBiasH2[2] << " " << x_[HEXABOT::L2_fs] << std::endl;
		  */

		  ofs << (double)t /20. << ", "
				  << osc[1].cpgPhase << "  " << osc[1].outPhase << "  " << -sin(osc[1].outPhase) << " "
				  << osc[1].footBias << " " <<  sDataSet.forceData[1] << " "
				  << ctr[1].l_toePos[0] << " " <<  ctr[1].l_toePos[1] << " " << ctr[1].l_toePos[2]<< " "
				  << ctr[1].tAngle << " " << ctr[1].cAngle << " " << ctr[1].fAngle << " "
				  << sDataSet.jData[1].tAngle << " " << sDataSet.jData[1].cAngle << " " << sDataSet.jData[1].fAngle << "           "

				  << osc[2].cpgPhase << "  " << osc[2].outPhase << "  " << -sin(osc[2].outPhase) << " "
				  << osc[2].footBias << " " <<  sDataSet.forceData[2] << " "
				  << ctr[2].l_toePos[0] << " " <<  ctr[2].l_toePos[1] << " " << ctr[2].l_toePos[2]<< " "
				  << sDataSet.jData[2].tAngle << " " << sDataSet.jData[2].cAngle << " " << sDataSet.jData[2].fAngle << "       "

				  << std::endl;
	  }
  }

  // Parameter logging --
  if(t == 1){
	  if(fileFlag){
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

  }

  // update step counter
  t++;

#ifdef DEBUG
	std::cout << "1step has finished , count " << t << std::endl;
#endif

}

/** stores the controller values to a given file. */
bool HexabotController::store(FILE* f) const {
  return true;
}

/** loads the controller values from a given file. */
bool HexabotController::restore(FILE* f) {
  return true;
}

/** get straight configulation **/
HexabotControllerConf HexabotController::getDefaultConf(){
  HexabotControllerConf c;

  // control Frequency
  c.controlFreq = 20.;

  // cnct param between two CPGs
  c.twoCpgCoef = 0.30;
  // diference in angle between two CPGs
  c.twoCPGAngleDiff = M_PI;

  // use Foot bias syustem or not
  //c.useFootBias = true;
  // start time (s)
  c.biasStartTime = 20;

  // set Period Changing mode
  c.PC_MODE = PC_IN_IN;
  c.is_oneSidePhaseModulation = false;

  // set Tfactor for phase reset continuous change
  c.TFactor = 1.;

  // configurate osc and trj param
  for(int i =0;i<CPGNUM;i++){
	  // for oscilation
	  c.oscConf[i].freq = 0.2;//(Hz)
	  c.oscConf[i].dutyRate = 0.8;
      c.oscConf[i].inhibiCoef = 0.2;//0.1;
	  c.oscConf[i].initialPhase = 0.;
  }
  c.oscConf[0].initialPhase = 0;
  c.oscConf[1].initialPhase = M_PI/3.;
  c.oscConf[2].initialPhase = 2*M_PI/3.;
  c.oscConf[3].initialPhase = M_PI + c.oscConf[0].initialPhase;
  c.oscConf[4].initialPhase = M_PI + c.oscConf[1].initialPhase;
  c.oscConf[5].initialPhase = M_PI + c.oscConf[2].initialPhase;

  // for traj

   // length from a base ]
   double l = 0.15;//0.15; // direction that is vertical to moving direction
   //double lx = -0.08;//- 0.08;//-0.05;// moving direction
   double h = - 0.12; // height of robot

   // right side
   for (int i=0; i<3; i++){
        c.trjConf[i].centerTrj = osg::Vec3d( l , 0 , h);
        c.trjConf[i].moveDirection = osg::Vec3d( 0, -1, 0.);
        c.trjConf[i].walkLength = 0.06;//0.1;
        c.trjConf[i].stanceHeight = 0.0;//0.01;
        c.trjConf[i].swingHeight = 0.06;//0.04;
    }

   // left side
   for (int i=0; i<3;i++){
        c.trjConf[i + 3].centerTrj = osg::Vec3d( l , 0, h);
        c.trjConf[i + 3].moveDirection = osg::Vec3d( 0, 1, 0.);
        c.trjConf[i + 3].walkLength = 0.06;//0.1;
        c.trjConf[i + 3].stanceHeight = 0.0;//0.01;
        c.trjConf[i + 3].swingHeight = 0.06;//0.04;
    }
    return c;
}


// This is the eFunction to analyze this oscillation from external viewer
// For Tempolary investigation
double HexabotController::eGetPhase(int ch){
    if(ch >= 0 && ch < CPGNUM){
		return osc[ch].cpgPhase;
	}else return -1;
}

// get oscillation data
HexabotController::oscSet HexabotController::eGetOscSet(int ch){
    if(ch >= 0 && ch < CPGNUM){
		return osc[ch];
	}else return osc[1];
}

// get control data
HexabotController::ctrSet HexabotController::eGetCtrSet(int ch){
    if(ch >= 0 && ch < CPGNUM){
		return ctr[ch];
	}else return ctr[1];
}

// get sensed data
HexabotController::sensedDataSet HexabotController::eGetSensedData(void){
	return sDataSet;
}

// get configuration of Hexabot controller
HexabotControllerConf HexabotController::eGetHexabotCtrConf(void){
	return conf;
}

// get leg is contacted
bool HexabotController::eIsLegContact(int ch){
    if(ch == 0){
        return (sDataSet.forceData[ch] > 0.1);
    }else if(ch == 1){
        return (sDataSet.forceData[ch] > 0.1 );//0.1);
    }else if(ch == 2){
        return (sDataSet.forceData[ch] > -0.0); //-0.03);
    }else if(ch == 3){
        return (sDataSet.forceData[ch] > 0.01);// 0.01);
    }else if(ch == 4){
        return (sDataSet.forceData[ch] > 0.1);//0.1);
    }else if(ch == 5){
        return (sDataSet.forceData[ch] > 0.1);//0.1);
    }else return false;
}

// get touch Down phase
double HexabotController::eGetTouchDownPhase(int ch){
    if(ch >= 0 && ch < CPGNUM){
        return osc[ch].touchDownPhase;
    }else return 0.;
}

// set Phase
bool HexabotController::eSetPhase(double phase, int ch){
    if(ch >= 0 && ch < CPGNUM){
        checkPhase(phase);
        osc[ch].cpgPhase = phase;
        return true;
    }else return false;
}

// set Phase Reset
bool HexabotController::eActivatePhaseReset_all(bool flag){
    for(int i =0; i<CPGNUM; i++){
        phaseResetFlag[i] = flag;
    }
    return true;
}

// set phaseRest each
bool HexabotController::eActivatePhaseReset_each(bool flag, int ch){
    if(ch >= 0 && ch < CPGNUM){
        phaseResetFlag[ch] = flag;
        return true;
    }else return false;
}

/*
bool HexabotController::eActivateOneSidePhaseModulation(bool flag){
    is_oneSidePhaseModulation = flag;
    return true;
}
*/

// For Phase Diff investigation
bool HexabotController::setObserverMode(){
	obsMode = true;
	return true;
}

// tell the address of this controller
bool HexabotController::addObject(HexabotController* obj){
	if(obsMode){
		mObject.push_back(obj);
		return true;
	}
	else return false;
}

// check the phase
void HexabotController::checkPhase(double& phase){
	if(phase >= 2. * M_PI) phase = phase - 2. * M_PI;
	else if(phase < 0) phase = phase + 2. * M_PI;
}

// inv Kinematics
bool HexabotController::clacInvKinematics(osg::Vec3d toePos, double& tAng, double& cAng, double& fAng){
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
	fAng = M_PI - acos((l1*l1 + l2*l2 - p*p)/(2.*l1*l2));

	return true;
}



