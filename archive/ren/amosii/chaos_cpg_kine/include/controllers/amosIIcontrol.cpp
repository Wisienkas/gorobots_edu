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
 *   $Log: tripodgate18dof.cpp,v $                                         *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#include <selforg/controller_misc.h>
#include "amosIIcontrol.h"
using namespace matrix;
using namespace std;

AmosIIControl::AmosIIControl() :
    AbstractController("AmosIIControl", "$Id: amosIIcontrol.cpp,v 0.1 $") {
  
  //---ADD YOUR initialization here---//
  
  t = 0; // step counter
      
  //---ADD YOUR initialization here---//
  
  //Call this function with your changeable parameters here//
  
  //Changeable to terminal
  // addParameter("WeightH1_H1", &control_your_extension.WeightH1_H1);
  
  //cpg for different legs
  addInspectableValue("cpg_fs", &control_adaptiveclimbing.cpg_output.at(0), "cpg_fs");
  addInspectableValue("cpg_RF", &control_adaptiveclimbing.cpg_leg.at(R0), "cpg_RF");
  addInspectableValue("cpg_RM", &control_adaptiveclimbing.cpg_leg.at(R1), "cpg_RM");
  addInspectableValue("cpg_RH", &control_adaptiveclimbing.cpg_leg.at(R2), "cpg_RH");
  addInspectableValue("cpg_LF", &control_adaptiveclimbing.cpg_leg.at(L0), "cpg_LF");
  addInspectableValue("cpg_LM", &control_adaptiveclimbing.cpg_leg.at(L1), "cpg_LM");
  addInspectableValue("cpg_LH", &control_adaptiveclimbing.cpg_leg.at(L2), "cpg_LH");
  
  //foot end trajectory for different legs
  addInspectableValue("Tj_RF_Y", &control_adaptiveclimbing.Tj_ForceFB.at(R0).y, "Tj_RF_Y");
  addInspectableValue("Tj_RF_Z", &control_adaptiveclimbing.Tj_ForceFB.at(R0).z, "Tj_RF_Z");
  addInspectableValue("Tj_RM_Y", &control_adaptiveclimbing.Tj_ForceFB.at(R1).y, "Tj_RM_Y");
  addInspectableValue("Tj_RM_Z", &control_adaptiveclimbing.Tj_ForceFB.at(R1).z, "Tj_RM_Z");
  addInspectableValue("Tj_RH_Y", &control_adaptiveclimbing.Tj_ForceFB.at(R2).y, "Tj_RH_Y");
  addInspectableValue("Tj_RH_Z", &control_adaptiveclimbing.Tj_ForceFB.at(R2).z, "Tj_RH_Z");
  addInspectableValue("Tj_LF_Y", &control_adaptiveclimbing.Tj_ForceFB.at(L0).y, "Tj_LF_Y");
  addInspectableValue("Tj_LF_Z", &control_adaptiveclimbing.Tj_ForceFB.at(L0).z, "Tj_LF_Z");
  addInspectableValue("Tj_LM_Y", &control_adaptiveclimbing.Tj_ForceFB.at(L1).y, "Tj_LM_Y");
  addInspectableValue("Tj_LM_Z", &control_adaptiveclimbing.Tj_ForceFB.at(L1).z, "Tj_LM_Z");
  addInspectableValue("Tj_LH_Y", &control_adaptiveclimbing.Tj_ForceFB.at(L2).y, "Tj_LH_Y");
  addInspectableValue("Tj_LH_Z", &control_adaptiveclimbing.Tj_ForceFB.at(L2).z, "Tj_LH_Z");
  
  //force sensor for foot contact point
  addInspectableValue("Fs_RF", &control_adaptiveclimbing.fs.at(R0), "Fs_RF");
  addInspectableValue("Fs_RM", &control_adaptiveclimbing.fs.at(R1), "Fs_RM");
  addInspectableValue("Fs_RH", &control_adaptiveclimbing.fs.at(R2), "Fs_RH");
  addInspectableValue("Fs_LF", &control_adaptiveclimbing.fs.at(L0), "Fs_LF");
  addInspectableValue("Fs_LM", &control_adaptiveclimbing.fs.at(L1), "Fs_LM");
  addInspectableValue("Fs_LH", &control_adaptiveclimbing.fs.at(L2), "Fs_LH");
  
  //Add edit parameter on terminal
  Configurable::addParameter("cin", &control_adaptiveclimbing.Control_input, /*minBound*/-10, /*maxBound*/10,
      "test discription");
  // prepare name;
  //Configurable::insertCVSInfo(name, "$RCSfile: amosIIcontrol.cpp,v $", "$Revision: 0.1 $");
  
}
;

AmosIIControl::~AmosIIControl() {
  
}

void AmosIIControl::init(int sensornumber, int motornumber, RandGen* randGen) {
  
  numbersensors = sensornumber;
  numbermotors = motornumber;
  x.resize(sensornumber);
  
}

/// performs one step (includes learning). Calulates motor commands from sensor inputs.
void AmosIIControl::step(const sensor* x_, int number_sensors, motor* y_, int number_motors) {
  
  assert(number_sensors == numbersensors);
  assert(number_motors == numbermotors);
  
  //0) Sensor inputs/scaling  ----------------
  
  //Angle sensors
  x.at(TR0_as) = x_[12];
  x.at(TR1_as) = x_[6];
  x.at(TR2_as) = x_[0];
  
  x.at(TL0_as) = x_[15];
  x.at(TL1_as) = x_[9];
  x.at(TL2_as) = x_[3];
  
  x.at(CR0_as) = x_[13];
  x.at(CR1_as) = x_[7];
  x.at(CR2_as) = x_[1];
  
  x.at(CL0_as) = x_[16];
  x.at(CL1_as) = x_[10];
  x.at(CL2_as) = x_[4];
  
  x.at(FR0_as) = x_[14];
  x.at(FR1_as) = x_[8];
  x.at(FR2_as) = x_[2];
  
  x.at(FL0_as) = x_[17];
  x.at(FL1_as) = x_[11];
  x.at(FL2_as) = x_[5];
  
  x.at(BJ_as) = x_[18];
  
  //Foot sensors
  
  x.at(R2_fs) = x_[19];
  x.at(L2_fs) = x_[20];
  x.at(R1_fs) = x_[21];
  x.at(L1_fs) = x_[22];
  x.at(R0_fs) = x_[23];
  x.at(L0_fs) = x_[24];
  
  //Adding more sensory inputs here
  
  //1) Neural preprocessing------------
  
  std::vector<double> x_prep = preprocessing_reflex.step_npp(x);
  
  //2) Neural learning and memory-----
  
  std::vector<double> memory_out = learningmemory_your_extension.step_nlm(x);
  
  //3) Neural locomotion control------
  
  //  manipulation of senors is possible here:
  //	x.at(TL0_as)=0;
  std::vector<double> tmp = control_adaptiveclimbing.step_nlc(x_prep, memory_out); //,/*Footinhibition = false, true*/false);
  //std:: vector<double> tmp = control_adaptiveclimbing.step_nlc(x,x_prep,memory_out,/*Footinhibition = false*/ false);
  
  /*************************************/

  //4) Motor postprocessing/scaling   ----------------
  /***Don't touch****Set Motor outputs begin *******************/
  y_[0] = tmp.at(TR2_m);
  y_[1] = tmp.at(CR2_m);
  y_[2] = tmp.at(FR2_m);
  
  y_[3] = tmp.at(TL2_m);
  y_[4] = tmp.at(CL2_m);
  y_[5] = tmp.at(FL2_m);
  
  y_[6] = tmp.at(TR1_m);
  y_[7] = tmp.at(CR1_m);
  y_[8] = tmp.at(FR1_m);
  
  y_[9] = tmp.at(TL1_m);
  y_[10] = tmp.at(CL1_m);
  y_[11] = tmp.at(FL1_m);
  
  y_[12] = tmp.at(TR0_m);
  y_[13] = tmp.at(CR0_m);
  y_[14] = tmp.at(FR0_m);
  
  y_[15] = tmp.at(TL0_m);
  y_[16] = tmp.at(CL0_m);
  y_[17] = tmp.at(FL0_m);
  y_[18] = tmp.at(BJ_m);
  
  /*******Set  Motor outputs end *******************/

  // update step counter
  t++;
}
;

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

