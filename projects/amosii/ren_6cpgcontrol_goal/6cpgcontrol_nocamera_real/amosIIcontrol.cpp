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



	addInspectableValue("CL1_fs",&control_adaptiveclimbing.fmodel_cml_output.at(1),"CL1_fs");
	addInspectableValue("CL2_fs",&control_adaptiveclimbing.fmodel_cml_output.at(2),"CL2_fs");
	addInspectableValue("CR2",&control_adaptiveclimbing.cr_output.at(2),"CR2");
	addInspectableValue("CR0",&control_adaptiveclimbing.m_pre.at(CR0_m),"CR0");
	addInspectableValue("CR0old",&control_adaptiveclimbing.postcrold.at(0),"CR2_fs");
	addInspectableValue("CR0new",&control_adaptiveclimbing.postcr.at(0),"CL0_fs");
	addInspectableValue("CL1_fs",&control_adaptiveclimbing.fmodel_cml_output.at(1),"CL1_fs");
	addInspectableValue("CL2_fs",&control_adaptiveclimbing.fmodel_cml_output.at(2),"CL2_fs");

	addInspectableValue("CR0",&control_adaptiveclimbing.m_pre.at(CR0_m),"CL1_fs");
	addInspectableValue("R0",&control_adaptiveclimbing.reflex_R_fs.at(0),"CL2_fs");
	addInspectableValue("CR1",&control_adaptiveclimbing.m_pre.at(CR1_m),"CL1_fs");
	addInspectableValue("R1",&control_adaptiveclimbing.reflex_R_fs.at(1),"CL2_fs");
	addInspectableValue("CR2",&control_adaptiveclimbing.m_pre.at(CR2_m),"CL1_fs");
	addInspectableValue("R2",&control_adaptiveclimbing.reflex_R_fs.at(2),"CL2_fs");
	addInspectableValue("CL0",&control_adaptiveclimbing.m_pre.at(CL0_m),"CL1_fs");
	addInspectableValue("L0",&control_adaptiveclimbing.reflex_L_fs.at(0),"CL2_fs");
	addInspectableValue("CL1",&control_adaptiveclimbing.m_pre.at(CL1_m),"CL1_fs");
	addInspectableValue("L1",&control_adaptiveclimbing.reflex_L_fs.at(1),"CL2_fs");
	addInspectableValue("CL2",&control_adaptiveclimbing.m_pre.at(CL2_m),"CL1_fs");
	addInspectableValue("L2",&control_adaptiveclimbing.reflex_L_fs.at(2),"CL2_fs");

	addInspectableValue("R0final",&control_adaptiveclimbing.fmodel_cmr_outputfinal.at(0),"CL2_fs");
	addInspectableValue("R1final",&control_adaptiveclimbing.fmodel_cmr_outputfinal.at(1),"CL1_fs");
	addInspectableValue("R2final",&control_adaptiveclimbing.fmodel_cmr_outputfinal.at(2),"CL2_fs");
	addInspectableValue("L0final",&control_adaptiveclimbing.fmodel_cml_outputfinal.at(0),"CL1_fs");
	addInspectableValue("L1final",&control_adaptiveclimbing.fmodel_cml_outputfinal.at(1),"CL2_fs");
	addInspectableValue("L2final",&control_adaptiveclimbing.fmodel_cml_outputfinal.at(2),"CL1_fs");
	addInspectableValue("error_R0",&control_adaptiveclimbing.acc_cmr_error_old.at(0),"error_R0");

	 addInspectableValue("R0_raw",&preprocessing_reflex.mappingsensor.at(R0_fs),"R0_raw");
	  addInspectableValue("output_preR0",&preprocessing_reflex.sensor_output.at(R0_fs),"output_preR0");



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


