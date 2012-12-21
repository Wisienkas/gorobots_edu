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
 *   $Log: testController.cpp,v $                                         *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#include <selforg/controller_misc.h>
#include <ode_robots/amosiisensormotordefinition.h>
#include "testController.h"

using namespace matrix;
using namespace std;
using namespace ASHIGARU;

TestController::TestController(const TestControllerConf& _conf)
: AbstractController("TestController", "$Id: testController.cpp,v 0.1 $"),
  conf(_conf) {
  t = 0;

  outputH1[0] = 0.001;
  outputH2[0] = 0.001;
  outputH1[1] = 0.8;
  outputH2[1] = -0.8;
  outputH1[2] = 0.1;
  outputH2[2] = 0.1;


  ofs.open("data2leg.txt");
  std::cout << "make a file"<< std::endl;
  ofs << "# This is the data of the neural bias and output " << std::endl;
  ofs << "# time(s),  outPutH1[1], outPutH2[1], bias[1], footContact[1], outPutH1[2], outPutH2[2], bias[2], footContact[2]" << std::endl;

  // prepare name;
  Configurable::insertCVSInfo(name, "$RCSfile: TestController.cpp,v $",
      "$Revision: 0.1 $");
}


TestController::~TestController(){
	ofs.close();
}

void TestController::init(int sensornumber, int motornumber, RandGen* randGen)
{
  // For initialize controller, we can restrict something
  //just for test, so I don't restrict any more
  assert(motornumber>=0);
}

/// performs one step (includes learning). Calculates motor commands from sensor
/// inputs.
void TestController::step(const sensor* x_, int number_sensors,
    motor* y_, int number_motors) {
  stepNoLearning(x_, number_sensors, y_, number_motors);
}

/// performs one step without learning. Calulates motor commands from sensor
/// inputs.
void TestController::stepNoLearning(const sensor* x_, int number_sensors,
    motor* y_, int number_motors) {

  //assert(number_sensors >= 18);
  //assert(number_motors >= 18);

  // set Parameter for SO(2) CPG
	// Notice ; it is temporal version, It should be changed to be readable
  static double alpha[3] = {1.6, 1.6, 1.6};
  static double phi[3] = {M_PI * 25./180., M_PI * 25./180., M_PI * 25./180.};
  static double normalBias[3] = {0.01, 0.01, 0.01};
  static double footBias[3] = {0., 0., 0.};
  static bool prvTouchF[3] = {false, false, false};
  static double rOutputH1[3];
  static double rOutputH2[3];
  static int count[3] = {0, 0, 0};

  // set Foot bias
	// 0:phase Reset, 1:Inhibitation, 2;ForceFeedBack, 3;Phase + Inhibi
  int phaseMode = 3;
  bool useFootBias = true;
  double coeff = 1.;
  if( useFootBias ){
	  for(int i =0 ; i < 3; i++){
		  // detect  whether it touched or not
		  bool touchF = (x_[ASHIGARU::L0_fs + i] > 0.01);
		  if(phaseMode == 0){
			  // When the leg toe touches the ground, we reset the phase.
			  if( touchF && !prvTouchF[i]){
				  footBias[i] =  - coeff * ( ( alpha[i] * cos(phi[i]) ) * outputH2[i] + ( - alpha[i] * sin(phi[i]) ) * outputH1[i] + normalBias[i]  );
			  }else{
				  footBias[i] = 0.;
			  }
		  }else if(phaseMode == 1){
			  // Input inhibitation
			  if(touchF){
				  footBias[i] = -0.2;
			  }else{
				  footBias[i] = 0.;
			  }

			  // freq. change mode
			  /*
			  if( touchF && !prvTouchF[i]){
				  double nextOut = ( ( alpha[i] * cos(phi[i]) ) * outputH2[i] + ( - alpha[i] * sin(phi[i]) ) * outputH1[i] + normalBias[i]  );
				  if(nextOut > 0.1){
					  phi[i] -= 0.01;
				  }else if(nextOut < -0.1){
					  phi[i] += 0.01;
				  }
			  }
			  */
		  }else if(phaseMode == 2){
			  // input force data oriented inihibitation
			  if(touchF){
				  footBias[i] = 0.2 -  x_[ASHIGARU::L0_fs + i];
			  }
		  }
		  else if(phaseMode == 3){
			  //Phase Reset + Inihibitation
			  if(count[i] > 0){
				  footBias[i] = -0.3;
				  count[i] -= 1;
			  }
			  else if( touchF && !prvTouchF[i]){
				  //calc next Step
				  double activityH1 = ( alpha[i] * cos(phi[i]) ) * outputH1[i] + ( alpha[i] * sin(phi[i]) ) * outputH2[i] + normalBias[i];
				  double activityH2 = ( alpha[i] * cos(phi[i]) ) * outputH2[i] + ( - alpha[i] * sin(phi[i]) ) * outputH1[i] + normalBias[i];// + footBias[i];//2.2 * footBias[i];
				  double outH1 = tanh(activityH1);
				  double outH2 = tanh(activityH2);

				  //Phase Reset
				  if(outH2 > 0. && outH1 > 0.){
					  footBias[i] =  - coeff * ( ( alpha[i] * cos(phi[i]) ) * outputH2[i] + ( - alpha[i] * sin(phi[i]) ) * outputH1[i] + normalBias[i]  );
				  }
				  //Inhibitation
				  else if(outH2 < 0){
					  footBias[i] = -0.3;
					  count[i] = (int)( (M_PI * 25./180.) /  phi[i] * 4.);
				  }else{
					  footBias[i] = 0.;
				  }
			  }else{
				  footBias[i] = 0.;
			  }
		  }

		  // update prvTouch
		  prvTouchF[i] = touchF;
	  }
  }


  // calculate CPG output
	// we use SO(2) simple CPG to generate oscillation pattern
	//  We apply CPG to each Leg
  for (int i=0;i<3;i++){
	  double activityH1 = ( alpha[i] * cos(phi[i]) ) * outputH1[i] + ( alpha[i] * sin(phi[i]) ) * outputH2[i] + normalBias[i];
	  double activityH2 = ( alpha[i] * cos(phi[i]) ) * outputH2[i] + ( - alpha[i] * sin(phi[i]) ) * outputH1[i] + normalBias[i] + footBias[i];//2.2 * footBias[i];

	  rOutputH1[i] = outputH1[i];
	  rOutputH2[i] = outputH2[i];

	  outputH1[i] = tanh(activityH1);
	  outputH2[i] = tanh(activityH2);

	  //if(footBias[i] == 0.){
	//	  rOutputH1[i] = outputH1[i];
	//	  rOutputH2[i] = outputH2[i];
	 // }
  }

  // Move the motors
  //  There are 2 types
  int mode = 0;

  if(mode == 0){
	  // This is the simple crawl version
	  //  it use only two legs, it is not interesting so much
	  double dir = -1.;
	  double dir2 = -1;// moving direction!!
	  double cWeight = 1.5;
	  double tWeight = 2.;
	  double fWeight = 3.;
	  double off3 = 0.5;
	  double off = 0.4;
	  double off2 = -0.35;

	    // conf fact = 0.3
	  // stopped Leg
	  y_[T0_m] = 0.;//outputH2 * conf.fact + conf.bias;
	  y_[C0_m] =  off; //outputH * conf.fact;//outputH2 * conf.fact;
	  y_[F0_m] =  off;//- cWeight * outputH2 * conf.fact * dir * dir2 ;//outputH2 * conf.fact + conf.bias;

	  // moving leg 1
	  y_[T1_m] =  -tWeight * outputH1[1] * conf.fact * dir * dir2 + off3;// 0.3;
	  y_[C1_m] =  -cWeight * outputH2[1] * conf.fact * dir + off;
	  y_[F1_m] = 0.;//outputH1 * conf.fact * conf.direction;

	  // moving leg 2
	  y_[T2_m] =  tWeight * outputH1[2] * conf.fact * dir * dir2 - off3;// - 0.3;
	  y_[C2_m] =  -cWeight *  outputH2[2] * conf.fact * dir + off;
	  y_[F2_m] =  0.;//-outputH1 * conf.fact * conf.direction;


	  /*
	  y_[9] = outputH1[1];
	  y_[10] = outputH2[1];
	  y_[11] = footBias[1];

	  y_[12] = outputH1[2];
	  y_[13] = outputH2[2];
	  y_[14] = footBias[2];
	   */

	  if(t > 50){
		  ofs << (double)t /10. << ", " << outputH1[1] << "  " << outputH2[1] << "  " << footBias[1] <<
			  "  " << x_[ASHIGARU::L1_fs] << "  " << outputH1[2] << "  " << outputH2[2] << "  " << footBias[2] <<
			  "  " << x_[ASHIGARU::L2_fs] << std::endl;
	  }
  }else if (mode == 1){
	  // This is the dynamic walking pattern (but it still make the body contact with ground)
		double dir = -1.;
		double dir2 = -1; // this is direction !!
		double cWeight = 1.5; // CT joint weight for L1, L2
		double tWeight = 2.; // TC joint weight for L1,L2
		double L0Weight = 3.; // weight for joints of LO
		double off3 = 0.; // offset for TC joint
		double off = 0.1; // offset for CT joint of L1, L2
		double off2 = -0.35; // offset for L0

		// send signal to joint inputs
		y_[T0_m] =   0.;//outputH2 * conf.fact + conf.bias;
		y_[C0_m] =   L0Weight * outputH1[0] * conf.fact * dir2 + off2;//outputH2 * conf.fact;
		y_[F0_m] = - L0Weight * outputH2[0] * conf.fact * dir * dir2 + off2;//outputH2 * conf.fact + conf.bias;

		y_[T1_m] =  tWeight * outputH1[1] * conf.fact * dir * dir2 + off3;// 0.3;
		y_[C1_m] =  cWeight * outputH2[1] * conf.fact * dir + off;
		y_[F1_m] = 	0.;//outputH1 * conf.fact * conf.direction;

		y_[T2_m] =  -tWeight * outputH1[2] * conf.fact * dir * dir2 - off3;// - 0.3;
		y_[C2_m] =  cWeight *  outputH2[2] * conf.fact * dir + off;
		y_[F2_m] = 	0.;//-outputH1 * conf.fact * conf.direction;
  }


  /*
  // This is the dynamic walking pattern (but it still make the body contact with ground)
  double dir = -1.;
  double dir2 = -1;
  double cWeight = 1.5;
  double tWeight = 2.;
  double fWeight = 3.;
  double off3 = 0.;
  double off = 0.1;
  double off2 = -0.35;

  // send signal to joint inputs
  y_[T0_m] = 0.;//outputH2 * conf.fact + conf.bias;
  y_[T1_m] =  tWeight * outputH1 * conf.fact * dir * dir2 + off3;// 0.3;
  y_[T2_m] =  -tWeight * outputH1 * conf.fact * dir * dir2 - off3;// - 0.3;
  y_[C0_m] =   3 * outputH1 * conf.fact * dir2 + off2;//outputH2 * conf.fact;
  y_[C1_m] =  cWeight * outputH2 * conf.fact * dir + off;
  y_[C2_m] =  cWeight *  outputH2 * conf.fact * dir + off;
  y_[F0_m] = - fWeight * outputH2 * conf.fact * dir * dir2 + off2;//outputH2 * conf.fact + conf.bias;
  y_[F1_m] = 0.;//outputH1 * conf.fact * conf.direction;
  y_[F2_m] = 0.;//-outputH1 * conf.fact * conf.direction;

  //  double a = 1.6;
  //  double phi = M_PI * 30./180.;

  */

  /*
  // This is the simple crawl version
  //  it use only two legs, it is not interesting
  double dir = -1.;
  double dir2 = -1;
  double cWeight = 1.5;
  double tWeight = 3.;
  double fWeight = 3.;
  double off3 = 0.;
  double off = 0.4;
  double off2 = -0.35;

  y_[T0_m] = 0.;//outputH2 * conf.fact + conf.bias;
  y_[T1_m] =  tWeight * outputH1 * conf.fact * dir * dir2;// 0.3;
  y_[T2_m] =  tWeight * outputH1 * conf.fact * dir * dir2;// - 0.3;
  y_[C0_m] =  off; //outputH * conf.fact;//outputH2 * conf.fact;
  y_[C1_m] =  cWeight * outputH2 * conf.fact * dir + off;
  y_[C2_m] = - cWeight *  outputH2 * conf.fact * dir + off;
  y_[F0_m] = off;//- cWeight * outputH2 * conf.fact * dir * dir2 ;//outputH2 * conf.fact + conf.bias;
  y_[F1_m] = 0.;//outputH1 * conf.fact * conf.direction;
  y_[F2_m] = 0.;//-outputH1 * conf.fact * conf.direction;
  */

  // update step counter
  t++;
}

/** stores the controller values to a given file. */
bool TestController::store(FILE* f) const {
  return true;
}

/** loads the controller values from a given file. */
bool TestController::restore(FILE* f) {
  return true;
}

/** get default configulation **/
TestControllerConf TestController::getDefaultConf() {
  TestControllerConf c;
  double a = 1.6;
  double phi = M_PI * 30./180.;
  c.WeightH1_H1 = a * cos(phi);
  c.WeightH2_H2 = a * cos(phi);
  c.WeightH1_H2 = a * sin(phi);
  c.WeightH2_H1 = - a * sin(phi);
  c.fact = 0.3; //0.7;
  c.direction = -1;
  c.bias = 0.0; //negative is legs up

  return c;
}

