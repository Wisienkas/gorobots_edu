
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
 *   $Log: testController5.cpp,v $                                         *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#include <selforg/controller_misc.h>
//#include <ode_robots/amosiisensormotordefinition.h>
#include "testController5.h"

using namespace matrix;
using namespace std;
using namespace ASHIGARU;

TestController5::TestController5(std::string name, double ini, const TestController5Conf& _conf)
: AbstractController("TestController5", "$Id: testController5.cpp,v 0.1 $"),obsMode(false),
  conf(_conf) {
  t = 0;

  for(int i = 0; i< 3; ++i){
	// Frequency
	oscW[i] = 1.;
	// phase
	phase[i] = 0.;
	// foot Bias
	footBias[i] = 0.;

	outputH1[i] = 1;

	outputH2[i] = 0.1;

	count[i] = 0.;
  }

  // setting for oscilatioon
	// feed back coefficiency
	fCoeff = 0.1;
	// phase diff
	pDiff = M_PI;
	// Inhibi Coeff
	//iCoeff = 0.1;


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
 // ofs << "# time(s),  outPutH1[1], outPutH2[1], finalOutT[1], finalOutC[1], biasH1[1], biasH2[1], footContact[1]"
 //		  ",  outPutH1[2], outPutH2[2], finalOutT[2], finalOutC[2], biasH1[2], biasH2[2], footContact[2]" << std::endl;
  ofs << "# time(s), outputH1[1], outputH2[1], footBias[1], footContact[1], outputH1[2], outputH2[2], footBias[2], footContac[2], phase[1], phase[2]" << std::endl;

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
  ofs3 << " time phase0 phase1 phase2 ... , diff 1-0 diff 2-1 .." << std::endl;


  // prepare name;
//  Configurable::insertCVSInfo(name, "$RCSfile: TestController5.cpp,v $",
//      "$Revision: 0.1 $");
}


TestController5::~TestController5() {
	ofs.close();
	ofs2.close();
	ofs3.close();

}

void TestController5::init(int sensornumber, int motornumber, RandGen* randGen)
{
  // For initialize controller, we can restrict something
  //just for test, so I don't restrict any more
  assert(motornumber>=0);
}

/// performs one step (includes learning). Calculates motor commands from sensor
/// inputs.
void TestController5::step(const sensor* x_, int number_sensors,
    motor* y_, int number_motors) {
  stepNoLearning(x_, number_sensors, y_, number_motors);
}

/// performs one step without learning. Calulates motor commands from sensor
/// inputs.
void TestController5::stepNoLearning(const sensor* x_, int number_sensors,
    motor* y_, int number_motors) {

  //############ Just test for CPG proposed by KOH ###########################

	bool useFootBias = true;
	double inhibiCoeff = 0.3;

	// calculatin of the Phase Reset and inhibitation
	if( useFootBias  && t > 150 ){
	  for(int s =0 ; s < CPGNUM; s++){
		  // detect  whether it touched or not
		  	bool touchF = (x_[ASHIGARU::L0_fs + s] > 0.01);

		  //Phase Reset + Inihibitation + force]
		  	 // Inhibitation duration
			if(count[s] > 0){
				footBias[s] = - iCoeff[s] * x_[ASHIGARU::L0_fs + s];
				int p = count[s];
				count[s] = p -  1;
			}

			 // Leg contact the ground !!
			else if( touchF && !prvTouchF[s]){
				// calculate this situation
				double ph = phase[s] + 0.1 * oscW[s];
				double outH2 = cos(ph);

				 //Phase Reset situation
				if(outH2 > 0. ){
				  footBias[s] = M_PI / 2. - ph;
				}
				 //Phase Inhibitation situation
				else if(outH2 < 0.){
				  iCoeff[s] = inhibiCoeff * ((ph - M_PI / 2.) / M_PI  );// temporal, inhibitation rate changes according to the time
				  footBias[s] = - iCoeff[s] * x_[ASHIGARU::L0_fs + s];
				  //footBias2[i] =   x_[ASHIGARU2::L0_fs + i];
				  count[s] = (int)( 2. * M_PI / (4. * oscW[s]) * 10. );
				}
				 // ELSE (there are no need to do phase reset or inhibitation)
				else{
				  footBias[s] = 0.;
				}
			 // ELSE
			}else{
				footBias[s] = 0.;
			}

			// update prvTouch
			prvTouchF[s] = touchF;
	  }
  }

	// ########### Calculation of CPG oscilation #################################################
	//  CPG
	  // calculate CPG output
		// we use  simple CPG to generate oscillation pattern
		//  We apply CPG to each Leg

	// connection parameter
	fCoeff =  0.05;

	phase[0] = phase[0] + 0.1 * oscW[0] + footBias[0];
	if(phase[0] > 2. * M_PI) phase[0] = phase[0] - 2. * M_PI;
	else if(phase[0] < 0) phase[0] = phase[0] + 2. * M_PI;

	phase[1] = phase[1] + 0.1 * oscW[1] - fCoeff * sin( phase[1] - phase[2] - pDiff ) + footBias[1];
	if(phase[1] > 2. * M_PI) phase[1] = phase[1] - 2. * M_PI;
	else if(phase[1] < 0) phase[1] = phase[1] + 2. * M_PI;

	phase[2] = phase[2] + 0.1 * oscW[2] - fCoeff * sin( phase[2] - phase[1] + pDiff ) + footBias[2];
	if(phase[2] > 2. * M_PI) phase[2] = phase[2] - 2. * M_PI;
	else if(phase[2] < 0) phase[2] = phase[2] + 2. * M_PI;

	//Phase Change
	// by this one,  I change the duration of stance and swing phase.
	double stanceRate = 0.9; // stance rate 0.5 - 1
	double newPhase;
	for(int s = 0; s<CPGNUM; s++){
		// ########### Phase Change situation
		// tPhase ; it is for easy calculating
		double tPhase = phase[s] - M_PI / 2.;
		if(tPhase > 2. * M_PI) tPhase = tPhase - 2. * M_PI;
		else if(tPhase < 0) tPhase = tPhase + 2. * M_PI;

		// stance phase conversion
		if( tPhase >= 0. &&  tPhase < (2. * M_PI * stanceRate) ){
			newPhase = tPhase / ( 2. * stanceRate ) + M_PI / 2.;
		}
		// swing phase conversion
		else{
			newPhase = (tPhase -  2. * M_PI * stanceRate  ) / ( 2. * (1 - stanceRate) ) + 3. * M_PI / 2.;
		}

		// ########### substitution
		outputH1[s] = sin(newPhase);
		outputH2[s] = cos(newPhase);
	}

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
		}else{
			c_output_post.at(s) = c_output.at(s);
		}
	}


	// Move the motors
	//  There are 2 types
	// This is the simple crawl version
	//  it use only two legs, it is not interesting so much
	double dir = -1.;
	double dir2 = -1;// moving direction!!
	double cWeight = 1.;
	double tWeight = 1.;//3.;
	double fWeight = 3.;
	double off3 = 0.5;//0.7
	double off = -0.1;
	double off2 = -0.35;

	// conf fact = 0.3
	// stopped Leg
	y_[T0_m] = 0.;//outputH2 * conf.fact + conf.bias;
	y_[C0_m] =  1.57; //outputH * conf.fact;//outputH2 * conf.fact;
	y_[F0_m] =  -1.57;//- cWeight * outputH2 * conf.fact * dir * dir2 ;//outputH2 * conf.fact + conf.bias;

	// moving leg 1
	  y_[T1_m] =  - tWeight * outputH1[1] * conf.fact * dir * dir2 + off3;// 0.3;
	  y_[C1_m] =  - cWeight * outputH2[1] * conf.fact * dir + off;
	//y_[T1_m] =  - tWeight * t_output.at(1) * conf.fact * dir * dir2 + off3;// 0.3;
	//y_[C1_m] =  - cWeight * c_output_post.at(1) * conf.fact * dir + off;
	y_[F1_m] = 0.;//outputH1 * conf.fact * conf.direction;

	// moving leg 2
	  y_[T2_m] =  tWeight * outputH1[2] * conf.fact * dir * dir2 - off3;// - 0.3;
	  y_[C2_m] =  - cWeight *  outputH2[2] * conf.fact * dir + off;
	//y_[T2_m] =  tWeight * t_output.at(2) * conf.fact * dir * dir2 - off3;// - 0.3;
	//y_[C2_m] =  - cWeight * c_output_post.at(2)  * conf.fact * dir + off;
	y_[F2_m] = 0.;//-outputH1 * conf.fact * conf.direction;

	if(conf.disLeg){
	  y_[T2_m] =  - off3;// - 0.3;
	  y_[C2_m] =   off + 0.5;
	  y_[F2_m] = 0.;//-outputH1 * conf.fact * conf.direction;
	}

  // logging
  if(t > 50){
	  /*// logger for PSN, VPN system
	  ofs << (double)t /10. << ", " << outputH1[1] << "  " << outputH2[1] << "  " << t_output.at(1) << "  " << c_output_post.at(1) << "  "
	  	  << footBiasH1[1] << " " << footBiasH2[1] << " " << x_[ASHIGARU2::L1_fs] << "  "
	  	  << outputH1[2] << "  " << outputH2[2] << "  " << t_output.at(2) << "  " << c_output_post.at(2) << "  "
	  	  << footBiasH1[2] << " " << footBiasH2[2] << " " << x_[ASHIGARU2::L2_fs] << std::endl;
	  */

	  ofs << (double)t /20. << ", " << outputH1[1] << "  " << outputH2[1] << "  "
	  	  	  << footBias[1] << " " <<  x_[ASHIGARU::L1_fs] << "  "
	  	  	  << outputH1[2] << "  " << outputH2[2] << "  "
	  	  	  << footBias[2] << " "  << x_[ASHIGARU::L2_fs] << " "
	  	  	  << phase[1] << " " <<  phase[2] << std::endl;

	  if(obsMode){
		  if(!mObject.empty()){
			  ofs3 << (double)t /20. << ", " ;
			  vector<TestController5*>::iterator it = mObject.begin();
			  while( it != mObject.end() )
			  {
				  double a = (*it)->getPhase(1);
				  ofs3 << a  << " " ;
				  ++it;
			  }

			  ofs3 <<  ",   " ;
			  double bVal = 0;
			  it = mObject.begin();
			  bVal = (*it)->getPhase(1);
			  ++it;

			  while( it != mObject.end() )
			  {
				  double a = (*it)->getPhase(1);

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
	  ofs2 << "Parameter for CPG " << endl;
	  ofs2 << " Type : " << "Simplified CPG" << endl;
	  ofs2 << " stanceRate: " << stanceRate << endl;
	  ofs2 << " InhibiCoeff:" << inhibiCoeff << " fCoeff:" << fCoeff << " pDiff:" << pDiff << endl;
	  for(int i =0;i<CPGNUM;i++){
		  ofs2 << " oscW[CPGNUM]:" << oscW[i]  <<  endl;
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
bool TestController5::store(FILE* f) const {
  return true;
}

/** loads the controller values from a given file. */
bool TestController5::restore(FILE* f) {
  return true;
}

/** get default configulation **/
TestController5Conf TestController5::getDefaultConf() {
  TestController5Conf c;
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

// For Tempolary investigation
double TestController5::getPhase(int ch){
	if(ch > 0 && ch < 3){
		return phase[ch];
	}else return -1;
}

// For Phase Diff investigation
bool TestController5::setObserverMode(){
	obsMode = true;
	return true;
}

// tell the address of this controller
bool TestController5::addObject(TestController5* obj){
	if(obsMode){
		mObject.push_back(obj);
		return true;
	}
	else return false;
}


