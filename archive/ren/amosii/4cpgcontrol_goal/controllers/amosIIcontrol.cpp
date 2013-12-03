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

AmosIIControl::AmosIIControl() :
    AbstractController("AmosIIControl", "$Id: amosIIcontrol.cpp,v 0.1 $") {

  //---ADD YOUR initialization here---//

  t = 0; // step counter

  //---ADD YOUR initialization here---//

  //Call this function with your changeable parameters here//

  //Changeable to terminal
  // addParameter("WeightH1_H1", &control_your_extension.WeightH1_H1);

  //added by guanjiao
//	addInspectableValue("Deviation",&control_adaptiveclimbing.alpha,"Deviation");
//	addInspectableValue("Deviation_x",&control_adaptiveclimbing.Deviation_of_x,"devia_x");
//	addInspectableValue("DistancetoGoal",&control_adaptiveclimbing.distance,"distance");
//	addInspectableValue("Delta_Devia_x",&control_adaptiveclimbing.delta_devia_x,"delta_devia_x");

  addInspectableValue("CPG0", &control_adaptiveclimbing.cpg_output.at(0), "CPG0");
  addInspectableValue("CPG1", &control_adaptiveclimbing.cpg_output1.at(0), "CPG1");
  addInspectableValue("CPG2", &control_adaptiveclimbing.cpg_output2.at(0), "CPG2");
  addInspectableValue("CPG3", &control_adaptiveclimbing.cpg_output3.at(0), "CPG3");
  addInspectableValue("CPG4", &control_adaptiveclimbing.cpg_output4.at(0), "CPG4");
  addInspectableValue("CPG5", &control_adaptiveclimbing.cpg_output5.at(0), "CPG5");

  addInspectableValue("psn0", &control_adaptiveclimbing.psn_output.at(11), "psn0");
  addInspectableValue("psn1", &control_adaptiveclimbing.psn_output1.at(11), "psn1");
  addInspectableValue("psn2", &control_adaptiveclimbing.psn_output2.at(11), "psn2");
  addInspectableValue("psn3", &control_adaptiveclimbing.psn_output3.at(11), "psn3");
  addInspectableValue("psn4", &control_adaptiveclimbing.psn_output4.at(11), "psn4");
  addInspectableValue("psn5", &control_adaptiveclimbing.psn_output5.at(11), "psn5");

  addInspectableValue("CR0", &control_adaptiveclimbing.m.at(CR0_m), "CR0");
  addInspectableValue("CR1", &control_adaptiveclimbing.m.at(CR1_m), "CR1");
  addInspectableValue("CR2", &control_adaptiveclimbing.m.at(CR2_m), "CR2");
  addInspectableValue("CL0", &control_adaptiveclimbing.m.at(CL0_m), "CL0");
  addInspectableValue("CL1", &control_adaptiveclimbing.m.at(CL1_m), "CL1");
  addInspectableValue("CL2", &control_adaptiveclimbing.m.at(CL2_m), "CL2");

  addInspectableValue("TC", &control_adaptiveclimbing.tr_output.at(2), "TC");
  addInspectableValue("CR", &control_adaptiveclimbing.cr_output.at(2), "CR");
  addInspectableValue("FTi", &control_adaptiveclimbing.fr_output.at(2), "FTi");
  addInspectableValue("CR0_fs", &control_adaptiveclimbing.fmodel_cmr_output.at(0), "CR0_fs");
  addInspectableValue("CR1_fs", &control_adaptiveclimbing.fmodel_cmr_output.at(1), "CR1_fs");
  addInspectableValue("CR2_fs", &control_adaptiveclimbing.fmodel_cmr_output.at(2), "CR2_fs");
  addInspectableValue("CL0_fs", &control_adaptiveclimbing.fmodel_cml_output.at(0), "CL0_fs");
  addInspectableValue("CL1_fs", &control_adaptiveclimbing.fmodel_cml_output.at(1), "CL1_fs");
  addInspectableValue("CL2_fs", &control_adaptiveclimbing.fmodel_cml_output.at(2), "CL2_fs");
  addInspectableValue("reflex_R0_fs", &control_adaptiveclimbing.reflex_R_fs.at(0), "reflex_R0_fs");
  addInspectableValue("reflex_R1_fs", &control_adaptiveclimbing.reflex_R_fs.at(1), "reflex_R1_fs");
  addInspectableValue("reflex_R2_fs", &control_adaptiveclimbing.reflex_R_fs.at(2), "reflex_R2_fs");
  addInspectableValue("reflex_L0_fs", &control_adaptiveclimbing.reflex_L_fs.at(0), "reflex_L0_fs");
  addInspectableValue("reflex_L1_fs", &control_adaptiveclimbing.reflex_L_fs.at(1), "reflex_L1_fs");
  addInspectableValue("reflex_L2_fs", &control_adaptiveclimbing.reflex_L_fs.at(2), "reflex_L2_fs");
  addInspectableValue("postCR0", &control_adaptiveclimbing.fmodel_cmr_outputfinal.at(0), "postCR0");
  addInspectableValue("postCR1", &control_adaptiveclimbing.fmodel_cmr_outputfinal.at(1), "postCR1");
  addInspectableValue("postCR2", &control_adaptiveclimbing.fmodel_cmr_outputfinal.at(2), "postCR2");
  addInspectableValue("postCL0", &control_adaptiveclimbing.fmodel_cml_outputfinal.at(0), "postCL0");
  addInspectableValue("postCL1", &control_adaptiveclimbing.fmodel_cml_outputfinal.at(1), "postCL1");
  addInspectableValue("postCL2", &control_adaptiveclimbing.fmodel_cml_outputfinal.at(2), "postCL2");
  addInspectableValue("errorR0_fs", &control_adaptiveclimbing.fmodel_cmr_error.at(0), "errorR0_fs");
  addInspectableValue("errorR1_fs", &control_adaptiveclimbing.fmodel_cmr_error.at(1), "errorR1_fs");
  addInspectableValue("errorR2_fs", &control_adaptiveclimbing.fmodel_cmr_error.at(2), "errorR2_fs");
  addInspectableValue("errorL0_fs", &control_adaptiveclimbing.fmodel_cml_error.at(0), "errorL0_fs");
  addInspectableValue("errorL1_fs", &control_adaptiveclimbing.fmodel_cml_error.at(1), "errorL1_fs");
  addInspectableValue("errorL2_fs", &control_adaptiveclimbing.fmodel_cml_error.at(2), "errorL2_fs");
  addInspectableValue("OrierrorR0_fs", &control_adaptiveclimbing.acc_cmr_error_old.at(0), "OrierrorR0_fs");
  addInspectableValue("OrierrorR1_fs", &control_adaptiveclimbing.acc_cmr_error_old.at(1), "OrierrorR1_fs");
  addInspectableValue("OrierrorR2_fs", &control_adaptiveclimbing.acc_cmr_error_old.at(2), "OrierrorR2_fs");
  addInspectableValue("OrierrorL0_fs", &control_adaptiveclimbing.acc_cml_error_old.at(0), "OrierrorL0_fs");
  addInspectableValue("OrierrorL1_fs", &control_adaptiveclimbing.acc_cml_error_old.at(1), "OrierrorL1_fs");
  addInspectableValue("OrierrorL2_fs", &control_adaptiveclimbing.acc_cml_error_old.at(2), "OrierrorL2_fs");
  addInspectableValue("WR0_fs", &control_adaptiveclimbing.fmodel_fmodel_cmr_w.at(0), "WR0_fs");
  addInspectableValue("WR1_fs", &control_adaptiveclimbing.fmodel_fmodel_cmr_w.at(1), "WR1_fs");
  addInspectableValue("WR2_fs", &control_adaptiveclimbing.fmodel_fmodel_cmr_w.at(2), "WR2_fs");
  addInspectableValue("lowpassR0_error", &control_adaptiveclimbing.lowpass_cmr__error_output.at(0), "lowpassR0_error");
  addInspectableValue("lowpassR1_error", &control_adaptiveclimbing.lowpass_cmr__error_output.at(1), "lowpassR1_error");
  addInspectableValue("R0_error", &control_adaptiveclimbing.fmodel_cmr_error.at(0), "R0_error");
  addInspectableValue("R1_error", &control_adaptiveclimbing.m_pre.at(CL0_m), "R1_error");
  addInspectableValue("AerrorR0_fs", &control_adaptiveclimbing.acc_cmr_error.at(0), "AerrorR0_fs");
  addInspectableValue("AerrorR1_fs", &control_adaptiveclimbing.acc_cmr_error.at(1), "AerrorR1_fs");
  addInspectableValue("AerrorR2_fs", &control_adaptiveclimbing.acc_cmr_error.at(2), "AerrorR2_fs");
  addInspectableValue("AerrorL0_fs", &control_adaptiveclimbing.acc_cml_error.at(0), "AerrorL0_fs");
  addInspectableValue("AerrorL1_fs", &control_adaptiveclimbing.acc_cml_error.at(1), "AerrorL1_fs");
  addInspectableValue("AerrorL2_fs", &control_adaptiveclimbing.acc_cml_error.at(2), "AerrorL2_fs");
  addInspectableValue("EorierrorR0_fs", &control_adaptiveclimbing.acc_cmr_error_old.at(0), "AerrorR0_fs");
  addInspectableValue("EorierrorR1_fs", &control_adaptiveclimbing.acc_cmr_error_old.at(1), "AerrorR1_fs");
  addInspectableValue("EorierrorR2_fs", &control_adaptiveclimbing.acc_cmr_error_old.at(2), "AerrorR2_fs");
  addInspectableValue("EorierrorL0_fs", &control_adaptiveclimbing.acc_cml_error_old.at(0), "AerrorL0_fs");
  addInspectableValue("EorierrorL1_fs", &control_adaptiveclimbing.acc_cml_error_old.at(1), "AerrorL1_fs");
  addInspectableValue("EorierrorL2_fs", &control_adaptiveclimbing.acc_cml_error_old.at(2), "AerrorL2_fs");
  addInspectableValue("EAerrorR0_fs", &control_adaptiveclimbing.error_cmr_elev.at(0), "EAerrorR0_fs");
  addInspectableValue("EAerrorR1_fs", &control_adaptiveclimbing.error_cmr_elev.at(1), "EAerrorR1_fs");
  addInspectableValue("EAerrorR2_fs", &control_adaptiveclimbing.error_cmr_elev.at(2), "EAerrorR2_fs");
  addInspectableValue("EAerrorL0_fs", &control_adaptiveclimbing.error_cml_elev.at(0), "EAerrorL0_fs");
  addInspectableValue("EAerrorL1_fs", &control_adaptiveclimbing.error_cml_elev.at(1), "EAerrorL1_fs");
  addInspectableValue("EAerrorL2_fs", &control_adaptiveclimbing.error_cml_elev.at(2), "EAerrorL2_fs");
  addInspectableValue("AEAerrorR0_fs", &control_adaptiveclimbing.acc_cmr_error_elev.at(0), "AEAerrorR0_fs");
  addInspectableValue("AEAerrorR1_fs", &control_adaptiveclimbing.acc_cmr_error_elev.at(1), "AEAerrorR1_fs");
  addInspectableValue("AEAerrorR2_fs", &control_adaptiveclimbing.acc_cmr_error_elev.at(2), "AEAerrorR2_fs");
  addInspectableValue("AEAerrorL0_fs", &control_adaptiveclimbing.acc_cml_error_elev.at(0), "AEAerrorL0_fs");
  addInspectableValue("AEAerrorL1_fs", &control_adaptiveclimbing.acc_cml_error_elev.at(1), "AEAerrorL1_fs");
  addInspectableValue("AEAerrorL2_fs", &control_adaptiveclimbing.acc_cml_error_elev.at(2), "AEAerrorL2_fs");
  addInspectableValue("OffsetR0_fs", &control_adaptiveclimbing.offset_ctr.at(0), "AEAerrorR0_fs");
  addInspectableValue("OffsetR1_fs", &control_adaptiveclimbing.offset_ctr.at(1), "AEAerrorR1_fs");
  addInspectableValue("OffsetR2_fs", &control_adaptiveclimbing.offset_ctr.at(2), "AEAerrorR2_fs");
  addInspectableValue("OffsetL0_fs", &control_adaptiveclimbing.offset_ctl.at(0), "AEAerrorL0_fs");
  addInspectableValue("OffsetL1_fs", &control_adaptiveclimbing.offset_ctl.at(1), "AEAerrorL1_fs");
  addInspectableValue("OffsetL2_fs", &control_adaptiveclimbing.offset_ctl.at(2), "AEAerrorL2_fs");
  addInspectableValue("CR0", &control_adaptiveclimbing.m_pre.at(CR0_m), "CR0");
  addInspectableValue("delayCR0", &control_adaptiveclimbing.m_pre_delay.at(CR0_m), "delayCR0");
  addInspectableValue("R0foot", &control_adaptiveclimbing.reflex_R_fs.at(0), "R0foot");
  addInspectableValue("fcnmodel0", &control_adaptiveclimbing.pred_r.at(0), "fcn model0");
  addInspectableValue("CR1", &control_adaptiveclimbing.m_pre.at(CR1_m), "CR1");
  addInspectableValue("delayCR1", &control_adaptiveclimbing.m_pre_delay.at(CR1_m), "delayCR1");
  addInspectableValue("R1foot", &control_adaptiveclimbing.reflex_R_fs.at(1), "R1foot");
  addInspectableValue("fcnmodel1", &control_adaptiveclimbing.pred_r.at(1), "fcn model1");
  addInspectableValue("CR2", &control_adaptiveclimbing.m_pre.at(CR2_m), "CR2");
  addInspectableValue("delayCR2", &control_adaptiveclimbing.m_pre_delay.at(CR2_m), "delayCR2");
  addInspectableValue("R2foot", &control_adaptiveclimbing.reflex_R_fs.at(2), "R2foot");
  addInspectableValue("fcnmodel2", &control_adaptiveclimbing.pred_r.at(2), "fcn model2");
  addInspectableValue("CL0", &control_adaptiveclimbing.m_pre.at(CL0_m), "CL0");
  addInspectableValue("delayCL0", &control_adaptiveclimbing.m_pre_delay.at(CL0_m), "delayCL0");
  addInspectableValue("L0foot", &control_adaptiveclimbing.reflex_L_fs.at(0), "L0foot");
  addInspectableValue("fcnmodel0", &control_adaptiveclimbing.pred_l.at(0), "fcn model0");
  addInspectableValue("CL1", &control_adaptiveclimbing.m_pre.at(CL1_m), "CL1");
  addInspectableValue("delayCL1", &control_adaptiveclimbing.m_pre_delay.at(CL1_m), "delayCL1");
  addInspectableValue("L1foot", &control_adaptiveclimbing.reflex_L_fs.at(1), "L1foot");
  addInspectableValue("fcnmodel1", &control_adaptiveclimbing.pred_l.at(1), "fcn model1");
  addInspectableValue("CL2", &control_adaptiveclimbing.m_pre.at(CL2_m), "CL2");
  addInspectableValue("delayCL2", &control_adaptiveclimbing.m_pre_delay.at(CL2_m), "delayCL2");
  addInspectableValue("L2foot", &control_adaptiveclimbing.reflex_L_fs.at(2), "L2foot");
  addInspectableValue("fcnmodel2", &control_adaptiveclimbing.pred_l.at(2), "fcn model2");

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

/// performs one step (includes learning). Calculates motor commands from sensor inputs.
void AmosIIControl::step(const sensor* x_, int number_sensors, motor* y_, int number_motors) {

  assert(number_sensors == numbersensors);
  assert(number_motors == numbermotors);

  //0) Sensor inputs/scaling  ----------------
  for (unsigned int i = 0; i < (numbersensors); i++) {
    x.at(i) = x_[i];
  }

  //1) Neural preprocessing-----------
  std::vector<double> x_prep = preprocessing_reflex.step_npp(x);

  //2) Neural learning and memory-----
  std::vector<double> memory_out = learningmemory_your_extension.step_nlm(x);

  //3) Neural locomotion control------

  //y = control_adaptiveclimbing.step_nlc(x,x_prep,memory_out,/*Footinhibition = false*/ false);
  y = control_adaptiveclimbing.step_nlc(x_prep, memory_out);

  //4) Motor postprocessing/scaling   ----------------

  for (unsigned int i = 0; i < (BJ_m + 1); i++) {
    y_[i] = 1 * y.at(i);
  }

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

