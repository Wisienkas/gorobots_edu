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
 *                                                                         *
 ***************************************************************************/

#include <selforg/controller_misc.h>
#include "amosIIcontrol.h"
using namespace matrix;
using namespace std;

AmosIIControl::AmosIIControl()
  : AbstractController("AmosIIControl", "$Id: amosIIcontrol.cpp,v 0.1 $"){

	//---ADD YOUR initialization here---//

	t=0;  // step counter

	//---ADD YOUR initialization here---//



	//Call this function with your changeable parameters here//

	//Changeable to terminal
	// addParameter("WeightH1_H1", &control_your_extension.WeightH1_H1);

	addInspectableValue("m_pre.atCR0",&control_adaptiveclimbing.m_pre.at(6),"m_preCR0");
	addInspectableValue("m_pre.atCR1",&control_adaptiveclimbing.m_pre.at(7),"m_preCR1");
	addInspectableValue("m_pre.atCR2",&control_adaptiveclimbing.m_pre.at(8),"m_preCR2");
	addInspectableValue("m_pre.atCL0",&control_adaptiveclimbing.m_pre.at(9),"m_preCL0");
	addInspectableValue("m_pre.atCL1",&control_adaptiveclimbing.m_pre.at(10),"m_preCL1");
	addInspectableValue("m_pre.atCL2",&control_adaptiveclimbing.m_pre.at(11),"m_preCL2");
	addInspectableValue("m_pre.atCR0",&control_adaptiveclimbing.m.at(TR0_m),"TR0_m");
	addInspectableValue("m_pre.atCR1",&control_adaptiveclimbing.m.at(CR0_m),"CR0_m");
	addInspectableValue("m_pre.atCR2",&control_adaptiveclimbing.m.at(FR0_m),"FR0");




  //Add edit parameter on terminal
  Configurable::addParameter("cin", &control_adaptiveclimbing.Control_input,  /*minBound*/ -10,  /*maxBound*/ 10,
		  "test discription" );
  // prepare name;
//  Configurable::insertCVSInfo(name, "$RCSfile: amosIIcontrol.cpp,v $",
//		  "$Revision: 0.1 $");


};

AmosIIControl::~AmosIIControl(){

}

void AmosIIControl::init(int sensornumber, int motornumber, RandGen* randGen){

  numbersensors=sensornumber;
  numbermotors=motornumber;
  x.resize(sensornumber);


}


/// performs one step (includes learning). Calculates motor commands from sensor inputs.
void AmosIIControl::step(const sensor* x_, int number_sensors, 
				    motor* y_, int number_motors){

  assert(number_sensors == numbersensors);
  assert(number_motors == numbermotors);

	//0) Sensor inputs/scaling  ----------------

	for(unsigned int i=0; i<(numbersensors);i++)
	{
		x.at(i) = x_[i];
	}


	//1) Neural preprocessing------------
	std:: vector<double> x_prep = preprocessing_reflex.step_npp(x);



	//2) Neural learning and memory-----
	std:: vector<double> memory_out = learningmemory_your_extension.step_nlm(x);


	//3) Neural locomotion control------

	//y = control_adaptiveclimbing.step_nlc(x,x_prep,memory_out,/*Footinhibition = false*/ false);
	//y = control_adaptiveclimbing.step_nlc(x_prep,memory_out);
	y = control_adaptiveclimbing.step_nlc(x_prep,x);




	//4) Motor postprocessing/scaling   ----------------

	for(unsigned int i=0; i<(BJ_m+1);i++)
	{
		y_[i] = 1*y.at(i);
	}

	// update step counter
	t++;
};

/** stores the controller values to a given file. */
bool AmosIIControl::store(FILE* f) const {
	//	std::cout << "hello \n";
	//	double bla = 10.0;
	//   fprintf(f, "%f", bla);

	//fprintf(f, "%f %f\n", preprocessing_reflex.preprosensor.at(L2_fs), preprocessing_reflex.preprosensor.at(L1_fs));
	//	Configurable::print(f, "");
	return true;
}

/** loads the controller values from a given file. */
bool AmosIIControl::restore(FILE* f) {
	//	Configurable::parse(f);
	return true;
}


