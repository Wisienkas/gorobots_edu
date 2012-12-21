
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
 *   $Log: testController4.cpp,v $                                         *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#include <selforg/controller_misc.h>
//#include <ode_robots/amosiisensormotordefinition.h>
#include "testController4.h"

using namespace matrix;
using namespace std;
using namespace ASHIGARU;

// Circle Buff
// constructor
CirBuff::CirBuff(){
	maxBuff = 100;
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
double CirBuff::getCurBuff(){
	return buff.at(currentBuffNum);
}

// getPrvBuffer
double CirBuff::getPrvBuff(int prvCount){
	int buffNum = currentBuffNum;
	buffNum -= prvCount;
	if(buffNum < 0)buffNum += maxBuff;

	if(buffNum >= 0 && buffNum < maxBuff)return buff.at(buffNum);
	else return -1;
}



TestController4::TestController4(std::string name, double ini, const TestController4Conf& _conf)
: AbstractController("TestController4", "$Id: testController4.cpp,v 0.1 $"),
  conf(_conf)
  {
  t = 0;
  // to get the address for Vector
  fBuff.resize(CPGNUM);

  for(int i = 0; i< 3; ++i){
	// Frequency
	ContM[i] = 0.08;

	// Coeff for osc
	w00[i] = 1.4;
	w01[i] = 0.18 + ContM[i];
	w10[i] = -0.18 - ContM[i];
	w11[i] = 1.4;

	// Bias
	normalBias[i] = 0.01;
	footBiasH1[i] = 0.;
	footBiasH2[i] = 0.;

	//delay Time
	delayTime[i] = 0.;
	countFlag[i] = false;
	count[i] = 0;


	if(i == 1){
		outputH1[i] = -1;
		outputH2[i] = 0.1;
	}else{
		outputH1[i] = 1;
		outputH2[i] = 0.1;
	}

#ifdef DEBUG_MODE
    debug[i] = 0.;
#endif

  }

	//Input vector size
	input.resize(5);
	  //Forward
	input.at(0) = 0; //All JOINTS inhibition = 0, 1 activate all joints = I1 in paper
	input.at(1) = 1; //FL0, FR1, FR2 control NOT USED
	input.at(2) = 1; //PSN control  = I2 in paper
	input.at(3) = -1; //turn left = 1,  = I3 in paper
	input.at(4) = -1; //turn right = 1,  = I4 in paper

	// ############# PSN
	psn_activity.resize(CPGNUM);
	for(unsigned int i=0; i<psn_activity.size(); i++){
		psn_activity.at(i).resize(12);
	}

	psn_output.resize(CPGNUM);
	for(unsigned int i=0; i<psn_output.size(); i++){
		psn_output.at(i).resize(12);
	}

	//---PSN weights
	psn_w.resize(12);
	for(unsigned int i=0; i<psn_w.size(); i++){
		psn_w.at(i).resize(12);
	}
	psn_w.at(2).at(0) = -5.0;
	psn_w.at(3).at(1) = -5.0;
	psn_w.at(4).at(0) = -5.0;
	psn_w.at(5).at(1) = -5.0;
	psn_w.at(6).at(2) =  0.5;
	psn_w.at(7).at(3) =  0.5;
	psn_w.at(8).at(4) =  0.5;
	psn_w.at(9).at(5) =  0.5;
	psn_w.at(10).at(6) = 3.0;
	psn_w.at(10).at(7) = 3.0;
	psn_w.at(11).at(8) = 3.0;
	psn_w.at(11).at(9) = 3.0;

	//network bias
	psn_bias.resize(3);
	psn_bias.at(0) =  1;
	psn_bias.at(1) =  0.5;
	psn_bias.at(2) = -1.35;

	//input2 to PSN connection weights
	psn_input2_w.resize(2);
	for(unsigned int i=0; i<psn_input2_w.size(); i++){
		psn_input2_w.at(i).resize(2);
	}
	psn_input2_w.at(0).at(0) = -1.0;
	psn_input2_w.at(1).at(0) = 1.0;

	//pCPG to PSN connection weights
	psn_pcpg_w.resize(14);
	for(unsigned int i=0; i<psn_pcpg_w.size(); i++){
		psn_pcpg_w.at(i).resize(14);
	}
	psn_pcpg_w.at(2).at(0) = 0.5;
	psn_pcpg_w.at(3).at(1) = 0.5;
	psn_pcpg_w.at(4).at(1) = 0.5;
	psn_pcpg_w.at(5).at(0) = 0.5;

	// ############# VRN
	// activity
	vrn_activity.resize(CPGNUM);
	for(unsigned int i=0; i<vrn_activity.size(); i++){
		vrn_activity.at(i).resize(14);
	}

	// output
	vrn_output.resize(CPGNUM);
	for(unsigned int i=0; i<vrn_output.size(); i++){
		vrn_output.at(i).resize(14);
	}

	// vrn weights
	vrn_w.resize(14);
	for(unsigned int i=0; i<vrn_w.size(); i++){
		vrn_w.at(i).resize(14);
	}
	vrn_w.at(4).at(0) =    1.7246;
	vrn_w.at(4).at(1) =    1.7246;
	vrn_w.at(5).at(0) =   -1.7246;
	vrn_w.at(5).at(1) =   -1.7246;
	vrn_w.at(6).at(0) =    1.7246;
	vrn_w.at(6).at(1) =   -1.7246;
	vrn_w.at(7).at(0) =   -1.7246;
	vrn_w.at(7).at(1) =    1.7246;
	vrn_w.at(8).at(2) =    1.7246;
	vrn_w.at(8).at(3) =    1.7246;
	vrn_w.at(9).at(2) =   -1.7246;
	vrn_w.at(9).at(3) =   -1.7246;
	vrn_w.at(10).at(2) =   1.7246;
	vrn_w.at(10).at(3) =  -1.7246;
	vrn_w.at(11).at(2) =  -1.7246;
	vrn_w.at(11).at(3) =   1.7246;
	vrn_w.at(12).at(4) =   0.5;
	vrn_w.at(12).at(5) =   0.5;
	vrn_w.at(12).at(6) =  -0.5;
	vrn_w.at(12).at(7) =  -0.5;
	vrn_w.at(13).at(8) =   0.5;
	vrn_w.at(13).at(9) =   0.5;
	vrn_w.at(13).at(10) = -0.5;
	vrn_w.at(13).at(11) = -0.5;
	//network bias
	vrn_bias =       -2.48285;

	//PSN to VRN connection weights
	vrn_psn_w.resize(14);
	for(unsigned int i=0; i<vrn_psn_w.size(); i++){
		vrn_psn_w.at(i).resize(14);
	}
	vrn_psn_w.at(0).at(11) = 1.75;
	vrn_psn_w.at(2).at(11) = 1.75;

	//input3 to VRN connection weight
	vrn_input3_w = 5;

	//input4 to VRN connection weight
	vrn_input4_w = 5;

	// ############### OUTPUT CPG
	t_activity.resize(CPGNUM);
	t_output.resize(CPGNUM);

	c_activity.resize(CPGNUM);
	c_output.resize(CPGNUM);
	c_outputold.resize(CPGNUM);
	c_output_post.resize(CPGNUM);

	f_activity.resize(CPGNUM);
	f_output.resize(CPGNUM);


  //file open
  std::string fileName;
  fileName = name;
  std::string a =  ".txt";
  fileName += a;
  char fileName2[30];
  strcpy(fileName2,fileName.c_str());
  ofs.open(fileName2);
  std::cout << "make a file"<< std::endl;
  ofs << "# This is the data of the neural bias and output and various CPG postprocessing " << std::endl;
  ofs << "# 1-time(s),  2-outPutH1[1], 3-outPutH2[1], 4-finalOutT[1], 5-finalOutC[1], 6-biasH1[1], 7-biasH2[1], 8-footContact[1], 9-sensedT[1], 10-sensedC[1], 11-delayed Force[1], 12-delayTime[1]"
		  << ",  13-outPutH1[2], 14-outPutH2[2], 15-finalOutT[2], 16-finalOutC[2], 17-biasH1[2], 18-biasH2[2], 19-footContact[2], 20-sensedT[2], 21-sensedC[2], 22-delayed Force[2], 23-delayTime[2]"
#ifdef DEBUG_MODE
		  << ",   24-debug[1], 25-debug[2] "
#endif
		  << std::endl;

  //file open
  fileName = name;
  a =  "Param.txt";
  fileName += a;
  strcpy(fileName2,fileName.c_str());
  ofs2.open(fileName2);
  std::cout << "make a Parameter file"<< std::endl;
  ofs2 << "# This contains the Parameters of this simulation " << std::endl;
  ofs2 << " " << std::endl;


  // prepare name;
  Configurable::insertCVSInfo(name, "$RCSfile: TestController4.cpp,v $",
      "$Revision: 0.1 $");
}


TestController4::~TestController4() {
	ofs.close();
}

void TestController4::init(int sensornumber, int motornumber, RandGen* randGen)
{
  // For initialize controller, we can restrict something
  //just for test, so I don't restrict any more
  assert(motornumber>=0);
}

/// performs one step (includes learning). Calculates motor commands from sensor
/// inputs.
void TestController4::step(const sensor* x_, int number_sensors,
    motor* y_, int number_motors) {
  stepNoLearning(x_, number_sensors, y_, number_motors);
}

/// performs one step without learning. Calulates motor commands from sensor
/// inputs.
void TestController4::stepNoLearning(const sensor* x_, int number_sensors,
    motor* y_, int number_motors) {

  //############ Just test for CPG proposed by KOH ###########################

	bool useFootBias = true;
	double inhibiCoeff = 0.2; // #param
	double fData[3];

	// calculatin of the Phase Reset and inhibitation
	if( useFootBias  && t > 150 ){
	  for(int s =0 ; s < CPGNUM; s++){
		  // logging ForceData
		  // By logging force data I want to compensate some delay in this system
		   fBuff[s].addBuff(x_[ASHIGARU::L0_fs + s]);
		   fData[s] = fBuff[s].getPrvBuff(delayTime[s]);

		  // detect  whether it touched or not
		  	bool touchF = (fData[s] > 0.01);

		  //Phase Reset + Inihibitation + force]
		  	 // Inhibitation duration
			if(count[s] > 0){
				footBiasH2[s] = - inhibiCoeff * fData[s];
				int p = count[s];
				count[s] = p -  1;

#ifdef DEBUG_MODE
	debug[s] = -0.2;
#endif
			}
			 // Leg contact the ground !!
			else if( touchF && !prvTouchF[s]){
				// calculate this situation
				double actH1 = w00[s] * outputH1[s] +  w01[s] * outputH2[s] + normalBias[s];
				double actH2 = w11[s] * outputH2[s] +  w10[s] * outputH1[s] + normalBias[s];// + cnctCoeff * outputH2[1];//2.2 * footBias[i];
				double outH1 = tanh(actH1);
				double outH2 = tanh(actH2);

				 //Phase Reset situation
				if(outH2 > 0. ){
				  footBiasH2[s] = - actH2;
				  footBiasH1[s] = 1. - actH1;

#ifdef DEBUG_MODE
	debug[s] = 0.4;
#endif

				}
				 //Phase Inhibitation situation
				else if(outH2 < 0.){
				  footBiasH2[s] = -  inhibiCoeff * fData[s];
				  //footBias2[i] =   x_[ASHIGARU2::L0_fs + i];
				  count[s] = (int)( 28 * 0.02 / ContM[s] );

#ifdef DEBUG_MODE
	debug[s] = -0.4;
#endif
				}
				 // ELSE (there are no need to do phase reset or inhibitation)
				else{
				  footBiasH1[s] = 0.;
				  footBiasH2[s] = 0.;

#ifdef DEBUG_MODE
	debug[s] = 0.;
#endif
				}
			 // ELSE
			}else{
				footBiasH1[s] = 0.;
				footBiasH2[s] = 0.;

#ifdef DEBUG_MODE
	debug[s] = 0.;
#endif
			}

			// update prvTouch
			prvTouchF[s] = touchF;
	  }
  }

	// ########### Calculation of CPG oscilation #################################################
	//  CPG
	  // calculate CPG output
		// we use KOH's simple CPG to generate oscillation pattern
		//  We apply CPG to each Leg

	// connection parameter
	double cnctCoeff = - 0.01;//5; // #param

	// CPG 0
	activityH1[0] = w00[0] * outputH1[0] +  w01[0] * outputH2[0] + normalBias[0] + footBiasH1[0];
	activityH2[0] = w11[0] * outputH2[0] +  w10[0] * outputH1[0] + normalBias[0] + footBiasH2[0];// + cnctCoeff * outputH2[1];//2.2 * footBias[i];
	outputH1[0] = tanh(activityH1[0]);
	outputH2[0] = tanh(activityH2[0]);

	// CPG 1
	activityH1[1] = w00[1] * outputH1[1] +  w01[1] * outputH2[1] + normalBias[1] + footBiasH1[1];
	activityH2[1] = w11[1] * outputH2[1] +  w10[1] * outputH1[1] + normalBias[1] + footBiasH2[1] + cnctCoeff * outputH2[2];// + cnctCoeff * outputH2[1];//2.2 * footBias[i];
	outputH1[1] = tanh(activityH1[1]);
	outputH2[1] = tanh(activityH2[1]);

    // CPG 2
	activityH1[2] = w00[2] * outputH1[2] +  w01[2] * outputH2[2] + normalBias[2] + footBiasH1[2];
	activityH2[2] = w11[2] * outputH2[2] +  w10[2] * outputH1[2] + normalBias[2] + footBiasH2[2] + cnctCoeff * outputH2[1];// + cnctCoeff * outputH2[1];//2.2 * footBias[i];
	outputH1[2] = tanh(activityH1[2]);
	outputH2[2] = tanh(activityH2[2]);

	/*
	for(int s = 0; s < CPGNUM; s++){
		activityH1[s] = w00[s] * outputH1[s] +  w01[s] * outputH2[s] + normalBias[s] + footBiasH1[s];
		activityH2[s] = w11[s] * outputH2[s] +  w10[s] * outputH1[s] + normalBias[s] + footBiasH2[s];// + cnctCoeff * outputH2[1];//2.2 * footBias[i];
		outputH1[s] = tanh(activityH1[s]);
		outputH2[s] = tanh(activityH2[s]);
	}
	*/

	// ############ Post Processing ###############################################################
	//***CPG post processing*****
	// By this One, we can get linearlized signal.
	//  The principle is that by using this, we draw line from the point whose y value is upper than a threshold to the point whose y value is lower than the threshold
	//  This line keenly depends on the threshold of the system
	//
	// But to this one , It needs one caluclation beforehand.
	// So, I do not know the effect of this situation to the phase reset

	for(int s = 0; s < CPGNUM; s++){
		//From CPG
		pcpg_step[s][0] = outputH1[s];
		pcpg_step[s][1] = outputH2[s];

		for(int i=0;i<2;i++){
			setold[s][i] = set[s][i];
			countupold[s][i] = countup[s][i];
			countdownold[s][i] = countdown[s][i];

		//1) Linear threshold transfer function neuron 1 , or called step function neuron//
		/********************************************************/

			if (pcpg_step[s][i]>=0.85) ////////////////////Intuitively select
			{
				set[s][i] = 1.0;
			}
			else if (pcpg_step[s][i]<0.85) ////////////////////Intuitively select
			{
				set[s][i] = -1.0;
			}

			diffset[s][i] = set[s][i]-setold[s][i]; // double

		//2) Count how many step of Swing
		/********************************************************/

			if (set[s][i] == 1.0){
				countup[s][i] = countup[s][i] + 1.0; //Delta x0 up
				countdown[s][i] = 0.0;
			}

			// Count how many step of Stance
			else if (set[s][i] == -1.0){
				countdown[s][i] = countdown[s][i] + 1.0; //Delta x0 down
				countup[s][i] = 0.0;
			}

		//3) Memorized the total steps of swing and stance
		/********************************************************/

			if (countup[s][i] == 0.0 && diffset[s][i] == -2.0 && set[s][i] == -1.0) deltaxup[s][i] = countupold[s][i];
			if (countdown[s][i] == 0.0 && diffset[s][i] == 2.0 && set[s][i] == 1.0) deltaxdown[s][i] = countdownold[s][i];

		//4) Comput y up and down !!!!
		/********************************************************/

			xup[s][i] =  countup[s][i];
			xdown[s][i] = countdown[s][i];

			////////////Scaling Slope Up calculation////////
			yup[s][i] = ( (2./deltaxup[s][i]) * xup[s][i]) - 1;
			////////////Scaling  Slope Down calculation//////
			ydown[s][i] = ( (-2./deltaxdown[s][i]) * xdown[s][i] ) + 1;

		//5) Combine y up and down !!!!
		/********************************************************/

			if (set[s][i] >= 0.0) pcpg_output[s][i] = yup[s][i];
			if (set[s][i] < 0.0) pcpg_output[s][i] = ydown[s][i];

			//********Limit upper and lower boundary
			if(pcpg_output[s][i]>1.0) pcpg_output[s][i] = 1.0;
			else if(pcpg_output[s][i]<-1.0) pcpg_output[s][i] = -1.0;
		}
	}
	//***CPG post processing*end*

	//################## PSN Processing #################################
	// By using this neural netwworks, signal becomes continuous
	//  By choosing Input signal's sign, we can change the movement direction
	//
	//
	for(int s = 0; s < CPGNUM; s++){
		psn_activity.at(s).at(0) = psn_input2_w.at(0).at(0) * input.at(2) + psn_bias.at(0);
		psn_activity.at(s).at(1) = psn_input2_w.at(1).at(0) * input.at(2);

		psn_activity.at(s).at(2) = psn_pcpg_w.at(2).at(0) * pcpg_output[s][0] + psn_w.at(2).at(0) * psn_output.at(s).at(0);
		psn_activity.at(s).at(3) = psn_pcpg_w.at(3).at(1) * pcpg_output[s][1] + psn_w.at(3).at(1) * psn_output.at(s).at(1);
		psn_activity.at(s).at(4) = psn_pcpg_w.at(4).at(1) * pcpg_output[s][1] + psn_w.at(4).at(0) * psn_output.at(s).at(0);
		psn_activity.at(s).at(5) = psn_pcpg_w.at(5).at(0) * pcpg_output[s][0] + psn_w.at(5).at(1) * psn_output.at(s).at(1);

		psn_activity.at(s).at(6) = psn_w.at(6).at(2) * psn_output.at(s).at(2) + psn_bias.at(1);
		psn_activity.at(s).at(7) = psn_w.at(7).at(3) * psn_output.at(s).at(3) + psn_bias.at(1);
		psn_activity.at(s).at(8) = psn_w.at(8).at(4) * psn_output.at(s).at(4) + psn_bias.at(1);
		psn_activity.at(s).at(9) = psn_w.at(9).at(5) * psn_output.at(s).at(5) + psn_bias.at(1);

		psn_activity.at(s).at(10) = psn_w.at(10).at(6) * psn_output.at(s).at(6) + psn_w.at(10).at(7) * psn_output.at(s).at(7) + psn_bias.at(2); // final output to motors
		psn_activity.at(s).at(11) = psn_w.at(11).at(8) * psn_output.at(s).at(8) + psn_w.at(11).at(9) * psn_output.at(s).at(9) + psn_bias.at(2); // final output to VRN

		for(unsigned int i=0; i<psn_output.at(s).size();i++){
			psn_output.at(s).at(i) = tanh(psn_activity.at(s).at(i));
		}
	}
	//********PSN end************

	//################ VRN Processing #############################
	// By using this Processing , the robot can change the direction,
	// Now, this system is no use for Ashigaru, But I applied it only for checking the movement
	//
	for(int s = 0; s < CPGNUM; s++){
		//VRN 1 (left)
		vrn_activity.at(s).at(0) = vrn_psn_w.at(0).at(11) * psn_output.at(s).at(11);
		vrn_activity.at(s).at(1) = vrn_input3_w * input.at(3);

		vrn_activity.at(s).at(4) = vrn_w.at(4).at(0) * vrn_output.at(s).at(0) + vrn_w.at(4).at(1) * vrn_output.at(s).at(1) + vrn_bias;
		vrn_activity.at(s).at(5) = vrn_w.at(5).at(0) * vrn_output.at(s).at(0) + vrn_w.at(5).at(1) * vrn_output.at(s).at(1) + vrn_bias;
		vrn_activity.at(s).at(6) = vrn_w.at(6).at(0) * vrn_output.at(s).at(0) + vrn_w.at(6).at(1) * vrn_output.at(s).at(1) + vrn_bias;
		vrn_activity.at(s).at(7) = vrn_w.at(7).at(0) * vrn_output.at(s).at(0) + vrn_w.at(7).at(1) * vrn_output.at(s).at(1) + vrn_bias;

		vrn_activity.at(s).at(12) = vrn_w.at(12).at(4) * vrn_output.at(s).at(4) + vrn_w.at(12).at(5) * vrn_output.at(s).at(5) + vrn_w.at(12).at(6) * vrn_output.at(s).at(6)
										+ vrn_w.at(12).at(7) * vrn_output.at(s).at(7); //Output to TL1,2,3

		//VRN 2 (right)
		vrn_activity.at(s).at(2) = vrn_psn_w.at(2).at(11) * psn_output.at(s).at(11);
		vrn_activity.at(s).at(3) = vrn_input4_w * input.at(4);

		vrn_activity.at(s).at(8) = vrn_w.at(8).at(2) * vrn_output.at(s).at(2) + vrn_w.at(8).at(3) * vrn_output.at(s).at(3) + vrn_bias;
		vrn_activity.at(s).at(9) = vrn_w.at(9).at(2) * vrn_output.at(s).at(2) + vrn_w.at(9).at(3) * vrn_output.at(s).at(3) + vrn_bias;
		vrn_activity.at(s).at(10) = vrn_w.at(10).at(2) * vrn_output.at(s).at(2) + vrn_w.at(10).at(3) * vrn_output.at(s).at(3) + vrn_bias;
		vrn_activity.at(s).at(11) = vrn_w.at(11).at(2) * vrn_output.at(s).at(2) + vrn_w.at(11).at(3) * vrn_output.at(s).at(3) + vrn_bias;

		vrn_activity.at(s).at(13) = vrn_w.at(13).at(8) * vrn_output.at(s).at(8) + vrn_w.at(13).at(9) * vrn_output.at(s).at(9) + vrn_w.at(13).at(10) * vrn_output.at(s).at(10)
										+ vrn_w.at(13).at(11) * vrn_output.at(s).at(11); //Output to TR1,2,3


		for(unsigned int i=0; i<vrn_output.at(s).size();i++)
		{
			vrn_output.at(s).at(i) = tanh(vrn_activity.at(s).at(i));
		}
	}
	//*******VRN end***************

	//################ Motor Neuron Processing #############################
	//  I applyed it to each leg of ASHIGARU
	//

	for(int s = 0; s < CPGNUM; s++){
		// storage of the previous neuron output
		c_outputold.at(s) = c_output.at(s);

		// calculate motor neuron
		// t neuron
		t_activity.at(s) = -2.5 * vrn_output.at(s).at(13) + 10*input.at(0);
		t_output.at(s) = tanh(t_activity.at(s));

		// c neuron
		c_activity.at(s) = 5.0 * psn_output.at(s).at(11) - 1.0 + 10*input.at(0);
		c_output.at(s) = tanh(c_activity.at(s));

		// f neuron
		f_activity.at(s) = 5.0 * psn_output.at(s).at(10)*input.at(1) - 0.5 + 10*input.at(0);
		f_output.at(s) = tanh(f_activity.at(s));

		// post processimg about c neuron
		//! I am afraid about this reasonability ???? It seem to be a little bit intuitionally!!!!!
		double diff = c_output.at(s) - c_outputold.at(s);
		if(diff < -0.0){
			c_output_post.at(s) = -1;
			if(countFlag[s]) timeCount[s]++;
		}else{
			c_output_post.at(s) = c_output.at(s);
			countFlag[s] = true;
			timeCount[s] = 0;
		}
		//time Delay calc
		if(outputH2[s] < 0){
			if(countFlag[s]){
				delayTime[s] = 0;//timeCount[s];
				countFlag[s] = false;
			}
		}

	}


	// Move the motors
	//  There are 2 types
	// This is the simple crawl version
	//  it use only two legs, it is not interesting so much
	// #param
	double dir = -1.;
	double dir2 = -1;// moving direction!!
	double cWeight = 1.;
	double tWeight = 1.;//3.;
	double fWeight = 3.;
	double off3 = 0.5;//0.7
	double off = 0.3;
	double off2 = -0.35;

	// conf fact = 0.3
	// stopped Leg
	y_[T0_m] = 0.;//outputH2 * conf.fact + conf.bias;
	y_[C0_m] =  1.57; //outputH * conf.fact;//outputH2 * conf.fact;
	y_[F0_m] =  -1.57;//- cWeight * outputH2 * conf.fact * dir * dir2 ;//outputH2 * conf.fact + conf.bias;

	// moving leg 1
	y_[T1_m] =  - tWeight * t_output.at(1) * conf.fact * dir * dir2 + off3;// 0.3;
	y_[C1_m] =  - cWeight * c_output_post.at(1) * conf.fact * dir + off;
	y_[F1_m] = 0.;//outputH1 * conf.fact * conf.direction;

	// moving leg 2
	y_[T2_m] =  tWeight * t_output.at(2) * conf.fact * dir * dir2 - off3;// - 0.3;
	y_[C2_m] =  - cWeight * c_output_post.at(2)  * conf.fact * dir + off;
	y_[F2_m] = 0.;//-outputH1 * conf.fact * conf.direction;

	if(conf.disLeg){
	  y_[T2_m] =  - off3;// - 0.3;
	  y_[C2_m] =   off + 0.5;
	  y_[F2_m] = 0.;//-outputH1 * conf.fact * conf.direction;
	}
	//y_[10] = outputH1[1];

  // logging
  if(t > 50){
	  ofs << (double)t /20. << ", " << outputH1[1] << "  " << outputH2[1] << "  " << t_output.at(1) << "  " << c_output_post.at(1) << "  "
	  	  << footBiasH1[1] << " " << footBiasH2[1] << " " << x_[ASHIGARU::L1_fs] << "  " << x_[ASHIGARU::T1_as] << "  " << x_[ASHIGARU::C1_as] << " "
	  	  << fData[1] << " " << delayTime[1] << " "
	  	  << outputH1[2] << "  " << outputH2[2] << "  " << t_output.at(2) << "  " << c_output_post.at(2) << "  "
	  	  << footBiasH1[2] << " " << footBiasH2[2] << " " << x_[ASHIGARU::L2_fs] <<  "  " << x_[ASHIGARU::T2_as] << "  " << x_[ASHIGARU::C2_as] << " "
          << fData[2] << " " << delayTime[2] << " "

#ifdef DEBUG_MODE
          << "    " << debug[1] << " " << debug[2]
#endif
	  	  <<  std::endl;
  }

  // Parameter logging --
  if(t == 1){
	  ofs2 << "Parameter for CPG " << endl;
	  ofs2 << " Type : " << "Improved CPG" << endl;
	  ofs2 << " InhibiCoeff:" << inhibiCoeff << " cnctCoeff:" << cnctCoeff << endl;
	  for(int i =0;i<CPGNUM;i++){
		  ofs2 << " ContM[CPGNUM]:" << ContM[i] << "  w00[]:" <<  w00[i] << "  w11[]:" <<  w11[i] <<
				  "  w01[]:" <<  w01[i] << " w10[]:" <<  w10[i] << "  normalBias[CPGNUM]:" << normalBias[i] <<  endl;
	  }
	  ofs2 << endl;

	  ofs2 << "Parameter for Servo Motor " << endl;
	  ofs2 << " dir:" << dir << "  dir2:" <<  dir2 << "  cWeight:" <<  cWeight <<
	  				  "  tWeight:" <<  tWeight << " fWeight:" <<  fWeight << "  off:" << off <<
	  				  "  off2:" << off2 << "  off3:" << off3 <<  endl;
  }




  // update step counter
  t++;
}

/** stores the controller values to a given file. */
bool TestController4::store(FILE* f) const {
  return true;
}

/** loads the controller values from a given file. */
bool TestController4::restore(FILE* f) {
  return true;
}

/** get default configulation **/
TestController4Conf TestController4::getDefaultConf() {
  TestController4Conf c;
  double a = 1.6;
  double phi = M_PI * 30./180.;
  c.WeightH1_H1 = a * cos(phi);
  c.WeightH2_H2 = a * cos(phi);
  c.WeightH1_H2 = a * sin(phi);
  c.WeightH2_H1 = - a * sin(phi);
  c.fact = 0.3; //0.7;
  c.direction = -1;
  c.bias = 0.0; //negative is legs up
  c.disLeg = false;

  return c;
}

