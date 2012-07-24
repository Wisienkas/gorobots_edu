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
#include <math.h>
#include "amosIIcontrol.h"
#include <ode_robots/amosII.h>
using namespace matrix;
using namespace std;

AmosIIControl::AmosIIControl() :
  AbstractController("AmosIIControl", "$Id: amosIIcontrol.cpp,v 0.1 $") {

  //---ADD YOUR initialization here---//

  t = 0; // step counter



  //---PLOT OPTIONS---//

  testing = false;
  plot_irlearning = false;   /*Plot IR learning for experiments or testing*/
  plot_preprofc = true;    /*Plot preprocessed foot contact sensor signals for testing*/
  plot_preproirs =false;    /*Plot preprocessed IR sensor signals for testing*/
  plot_preproirleg = false; /*Plot preprocessed IR leg sensor signals for testing*/
  plot_fmodel_ctr = false;  /*Plot fmodel signals of ctr joints for testing*/
  plot_reflex_fs = false;   /*Plot fs signals used for reflexes*/
  plot_post_ctr = false;    /*Plot postprocessed ctr signals for testing*/
  plot_fmodel_errors = false; /*Plot errors of fmodel*/
  plot_fmodel_w = false;    /*Plot weights of fmodel*/
  plot_ussensor_obstacle_avoidance= true;/*Plot US_sensors for obstacle avoidance*/
  plot_reversegear = false;  /*Plot obstacle avoidance behavior of robot*/
  plot_nlc = false;       /*Plot nlc control signals*/
  plot_testbjc = true;    /*Plot BJC for testing*/
  plot_cpg = false;    /*Plot CPG for testing*/
  plot_fmodel_counter = false;
  plot_testlaser=true;

  //Call this function with your changeable parameters here//

  //Changeable to terminal
  // addParameter("WeightH1_H1", &control_your_extension.WeightH1_H1);


  //1) Neural preprocessing plotting-----

  if(testing)
  {
    addInspectableValue("CPGtest0",&control_adaptiveclimbing.cpg_output.at(0),"CPGtest0");
    addInspectableValue("CPGtest1",&control_adaptiveclimbing.cpg_output.at(1),"CPGtest1");
    addInspectableValue("PSN0",&control_adaptiveclimbing.psn_output.at(10),"PSN0");
    addInspectableValue("PSN1",&control_adaptiveclimbing.psn_output.at(11),"PSN1");
    addInspectableValue("VRN0",&control_adaptiveclimbing.vrn_output.at(12),"VRN0");
    addInspectableValue("VRN1",&control_adaptiveclimbing.vrn_output.at(13),"VRN1");

  }

  if(plot_preprofc)
  {
    addInspectableValue("R0_fs",&preprocessing_learning.preprosensor.at(R0_fs).at(0),"R0_fs");
    addInspectableValue("R1_fs",&preprocessing_learning.preprosensor.at(R1_fs).at(0),"R1_fs");
    addInspectableValue("R2_fs",&preprocessing_learning.preprosensor.at(R2_fs).at(0),"R2_fs");
    addInspectableValue("L0_fs",&preprocessing_learning.preprosensor.at(L0_fs).at(0),"L0_fs");
    addInspectableValue("L1_fs",&preprocessing_learning.preprosensor.at(L1_fs).at(0),"L1_fs");
    addInspectableValue("L2_fs",&preprocessing_learning.preprosensor.at(L2_fs).at(0),"L2_fs");
    addInspectableValue("BJ_as",&preprocessing_learning.preprosensor.at(BJ_as).at(0),"L1_fs");
  }
  if(plot_preproirs)
  {
    addInspectableValue("R0_irs",&preprocessing_learning.preprosensor.at(R0_irs).at(0),"R0_irs");
    addInspectableValue("R1_irs",&preprocessing_learning.preprosensor.at(R1_irs).at(0),"R1_irs");
    addInspectableValue("R2_irs",&preprocessing_learning.preprosensor.at(R2_irs).at(0),"R2_irs");
    addInspectableValue("L0_irs",&preprocessing_learning.preprosensor.at(L0_irs).at(0),"L0_irs");
    addInspectableValue("L1_irs",&preprocessing_learning.preprosensor.at(L1_irs).at(0),"L1_irs");
    addInspectableValue("L2_irs",&preprocessing_learning.preprosensor.at(L2_irs).at(0),"L2_irs");
  }


  addInspectableValue("CR0_outputfinal", &control_adaptiveclimbing.fmodel.at(CR0_m)->outputfinal, "CL1_fs");
  addInspectableValue("CR1_outputfinal", &control_adaptiveclimbing.fmodel.at(CR1_m)->outputfinal, "CL1_fs");
  addInspectableValue("CR2_outputfinal", &control_adaptiveclimbing.fmodel.at(CR2_m)->outputfinal, "CL1_fs");
  addInspectableValue("CL0_outputfinal", &control_adaptiveclimbing.fmodel.at(CL0_m)->outputfinal, "CL1_fs");
  addInspectableValue("CL1_outputfinal", &control_adaptiveclimbing.fmodel.at(CL1_m)->outputfinal, "CL1_fs");
  addInspectableValue("CL2_outputfinal", &control_adaptiveclimbing.fmodel.at(CL2_m)->outputfinal, "CL1_fs");

  addInspectableValue("TR0_outputfinal", &control_adaptiveclimbing.fmodel.at(TR0_m)->outputfinal, "TL1_fs");
  addInspectableValue("TR1_outputfinal", &control_adaptiveclimbing.fmodel.at(TR1_m)->outputfinal, "TL1_fs");
  addInspectableValue("TR2_outputfinal", &control_adaptiveclimbing.fmodel.at(TR2_m)->outputfinal, "TL1_fs");
  addInspectableValue("TL0_outputfinal", &control_adaptiveclimbing.fmodel.at(TL0_m)->outputfinal, "TL1_fs");
  addInspectableValue("TL1_outputfinal", &control_adaptiveclimbing.fmodel.at(TL1_m)->outputfinal, "TL1_fs");
  addInspectableValue("TL2_outputfinal", &control_adaptiveclimbing.fmodel.at(TL2_m)->outputfinal, "TL1_fs");

  addInspectableValue("FR0_outputfinal", &control_adaptiveclimbing.fmodel.at(FR0_m)->outputfinal, "FL1_fs");
  addInspectableValue("FR1_outputfinal", &control_adaptiveclimbing.fmodel.at(FR1_m)->outputfinal, "FL1_fs");
  addInspectableValue("FR2_outputfinal", &control_adaptiveclimbing.fmodel.at(FR2_m)->outputfinal, "FL1_fs");
  addInspectableValue("FL0_outputfinal", &control_adaptiveclimbing.fmodel.at(FL0_m)->outputfinal, "FL1_fs");
  addInspectableValue("FL1_outputfinal", &control_adaptiveclimbing.fmodel.at(FL1_m)->outputfinal, "FL1_fs");
  addInspectableValue("FL2_outputfinal", &control_adaptiveclimbing.fmodel.at(FL2_m)->outputfinal, "FL1_fs");





//2) Neural learning and memory plotting-----



  if(plot_irlearning)
    {
      addInspectableValue("IR_sensor_outR", &preprocessing_learning.sensor_output.at(25), "IR_learn_rho1_R");
      addInspectableValue("IR_sensor_outL",&preprocessing_learning.sensor_output.at(26),"IR_learn_rho1_L");
      addInspectableValue("IR_Predic_output_R",&preprocessing_learning.ir_predic_output.at(25),"IR_Predic_output_R");
      addInspectableValue("IR_dReflex_output_R",&preprocessing_learning.ir_reflex_output.at(25),"IR_dReflex_output_R");
      addInspectableValue("IR_Sensor_output_L",&preprocessing_learning.ir_predic_output.at(26),"IR_Sensor_output_L");
      addInspectableValue("IR_dReflex_output_L",&preprocessing_learning.ir_reflex_output.at(26),"IR_dReflex_output_L");
      addInspectableValue("IR_learn_rho1_R",&preprocessing_learning.rho1.at(25),"IR_learn_rho1_R");
      addInspectableValue("IR_learn_rho1_L",&preprocessing_learning.rho1.at(26),"IR_learn_rho1_L");
      addInspectableValue("IR_learnoutput_R",&preprocessing_learning.irlearn_output.at(25),"IR_learnoutput_R");
      addInspectableValue("IR_learnoutput_L",&preprocessing_learning.irlearn_output.at(26),"IR_learnoutput_L");
      addInspectableValue("IR_learnoutput_R_prolong",&preprocessing_learning.irlearn_output_prolong.at(25),"IR_learnoutput_R");
      addInspectableValue("IR_learnoutput_L_prolong",&preprocessing_learning.irlearn_output_prolong.at(26),"IR_learnoutput_L");
      addInspectableValue("IR_learnoutput_R_delay",&preprocessing_learning.preprosensor.at(FR_us).at(0),"IR_learnoutput_R");
      addInspectableValue("IR_learnoutput_L_delay",&preprocessing_learning.preprosensor.at(FL_us).at(0),"IR_learnoutput_L");
    }

  //3) Neural locomotion control plotting------

  if(plot_cpg)
  {
    addInspectableValue("CPG0",&control_adaptiveclimbing.cpg_output.at(0),"CPG0");
    addInspectableValue("CPG1",&control_adaptiveclimbing.cpg_output.at(1),"CPG1");
  }

  if(plot_fmodel_ctr)
  {
    addInspectableValue("input",&control_adaptiveclimbing.fmodel.at(CR0_m)->input,"CR0_fs");
    addInspectableValue("output",&control_adaptiveclimbing.fmodel_output.at(CR0_m),"CR0_fs");
    addInspectableValue("outputfinal",&control_adaptiveclimbing.fmodel_outputfinal.at(CR0_m),"CL0_fs");
    addInspectableValue("lerror",&control_adaptiveclimbing.fmodel_lerror.at(CR0_m),"CR1_fs");
    addInspectableValue("error",&control_adaptiveclimbing.fmodel_error.at(CR0_m),"CR2_fs");
    addInspectableValue("accerror",&control_adaptiveclimbing.acc_error.at(CR0_m),"CL2_fs");
    addInspectableValue("m_input",&control_adaptiveclimbing.fmodel.at(CR1_m)->input,"CR0_fs");
    addInspectableValue("m_output",&control_adaptiveclimbing.fmodel_output.at(CR1_m),"CR0_fs");
    addInspectableValue("m_outputfinal",&control_adaptiveclimbing.fmodel_outputfinal.at(CR1_m),"CL0_fs");
    addInspectableValue("m_lerror",&control_adaptiveclimbing.fmodel_lerror.at(CR1_m),"CR1_fs");
    addInspectableValue("m_error",&control_adaptiveclimbing.fmodel_error.at(CR1_m),"CR2_fs");
    addInspectableValue("m_accerror",&control_adaptiveclimbing.acc_error.at(CR1_m),"CL2_fs");
  }
  if(plot_reflex_fs)
  {
    addInspectableValue("reflex_R0_fs",&control_adaptiveclimbing.reflex_fs.at(R0_fs),"reflex_R0_fs");
    addInspectableValue("reflex_R1_fs",&control_adaptiveclimbing.reflex_fs.at(R1_fs),"reflex_R1_fs");
    addInspectableValue("reflex_R2_fs",&control_adaptiveclimbing.reflex_fs.at(R2_fs),"reflex_R2_fs");
    addInspectableValue("reflex_L0_fs",&control_adaptiveclimbing.reflex_fs.at(L0_fs),"reflex_L0_fs");
    addInspectableValue("reflex_L1_fs",&control_adaptiveclimbing.reflex_fs.at(L1_fs),"reflex_L1_fs");
    addInspectableValue("reflex_L2_fs",&control_adaptiveclimbing.reflex_fs.at(L2_fs),"reflex_L2_fs");
  }
  if(plot_post_ctr)
  {
    addInspectableValue("ctr_R0_fs", &control_adaptiveclimbing.m_pre.at(CR0_m), "ctr_R0_fs");
    addInspectableValue("ctr_R1_fs", &control_adaptiveclimbing.m_pre.at(CR1_m), "ctr_R1_fs");
    addInspectableValue("ctr_R2_fs", &control_adaptiveclimbing.m_pre.at(CR2_m), "ctr_R2_fs");
    addInspectableValue("ctr_L0_fs", &control_adaptiveclimbing.m_pre.at(CL0_m), "ctr_L0_fs");
    addInspectableValue("ctr_L1_fs", &control_adaptiveclimbing.m_pre.at(CL1_m), "ctr_L1_fs");
    addInspectableValue("ctr_L2_fs", &control_adaptiveclimbing.m_pre.at(CL2_m), "ctr_L2_fs");
  }
  if(plot_fmodel_errors)
  {
    addInspectableValue("errorR0_fs",&control_adaptiveclimbing.fmodel_error.at(CR0_m),"errorR0_fs");
    addInspectableValue("errorR1_fs",&control_adaptiveclimbing.fmodel_error.at(CR1_m),"errorR1_fs");
    addInspectableValue("errorR2_fs",&control_adaptiveclimbing.fmodel_error.at(CR2_m),"errorR2_fs");
    addInspectableValue("errorL0_fs",&control_adaptiveclimbing.fmodel_error.at(CL0_m),"errorL0_fs");
    addInspectableValue("errorL1_fs",&control_adaptiveclimbing.fmodel_error.at(CL1_m),"errorL1_fs");
    addInspectableValue("errorL2_fs",&control_adaptiveclimbing.fmodel_error.at(CL2_m),"errorL2_fs");
  }
  if(plot_fmodel_counter)
  {
    addInspectableValue("counterR0_fs",&control_adaptiveclimbing.fmodel_counter.at(CR0_m),"counterR0_fs");
    addInspectableValue("counterR1_fs",&control_adaptiveclimbing.fmodel_counter.at(CR1_m),"counterR1_fs");
    addInspectableValue("counterR2_fs",&control_adaptiveclimbing.fmodel_counter.at(CR2_m),"counterR2_fs");
    addInspectableValue("counterL0_fs",&control_adaptiveclimbing.fmodel_counter.at(CL0_m),"counterL0_fs");
    addInspectableValue("counterL1_fs",&control_adaptiveclimbing.fmodel_counter.at(CL1_m),"counterL1_fs");
    addInspectableValue("counterL2_fs",&control_adaptiveclimbing.fmodel_counter.at(CL2_m),"counterL2_fs");
  }
  if(plot_fmodel_w)
  {
    addInspectableValue("WR0_fs",&control_adaptiveclimbing.fmodel_fmodel_w.at(CR0_m),"WR0_fs");
    addInspectableValue("WR1_fs",&control_adaptiveclimbing.fmodel_fmodel_w.at(CR1_m),"WR1_fs");
    addInspectableValue("WR2_fs",&control_adaptiveclimbing.fmodel_fmodel_w.at(CR2_m),"WR2_fs");
    addInspectableValue("WL0_fs",&control_adaptiveclimbing.fmodel_fmodel_w.at(CL0_m),"WL0_fs");
    addInspectableValue("WL1_fs",&control_adaptiveclimbing.fmodel_fmodel_w.at(CL1_m),"WL1_fs");
    addInspectableValue("WL2_fs",&control_adaptiveclimbing.fmodel_fmodel_w.at(CL2_m),"WL2_fs");
  }

  if(plot_nlc)
  {
    addInspectableValue("MODULE1OUTPUT1",&control_adaptiveclimbing.cpg_output.at(0),"MODULE1OUTPUT1");
    addInspectableValue("MODULE1OUTPUT2",&control_adaptiveclimbing.cpg_output.at(1),"MODULE1OUTPUT2");
    addInspectableValue("MODULE2OUTPUT1",&control_adaptiveclimbing.pcpg_output.at(0),"MODULE2OUTPUT1");
    addInspectableValue("MODULE2OUTPUT2",&control_adaptiveclimbing.pcpg_output.at(1),"MODULE2OUTPUT2");
    addInspectableValue("MODULE3OUTPUT1",&control_adaptiveclimbing.psn_output.at(10),"MODULE3OUTPUT1");
    addInspectableValue("MODULE3OUTPUT2",&control_adaptiveclimbing.psn_output.at(11),"MODULE3OUTPUT2");
    addInspectableValue("MODULE4OUTPUT1",&control_adaptiveclimbing.vrn_output.at(12),"MODULE4OUTPUT1");
    addInspectableValue("MODULE4OUTPUT2",&control_adaptiveclimbing.vrn_output.at(13),"MODULE4OUTPUT2");
    addInspectableValue("MODULE8OUTPUTpre",&control_adaptiveclimbing.m_pre.at(TR1_m),"MODULE8OUTPUTpre");
    addInspectableValue("MODULE8OUTPUT",&control_adaptiveclimbing.m_reflex.at(TR1_m),"MODULE8OUTPUT");
  }

  if(plot_ussensor_obstacle_avoidance)
  {
	  //switched FR FL, cause output neurons are twisted!
	  addInspectableValue("FL_us",&preprocessing_learning.preprosensor.at(FR_us).at(1),"FL_us");
	  addInspectableValue("FR_us",&preprocessing_learning.preprosensor.at(FL_us).at(1),"FR_us");
	  addInspectableValue("input(3)",&control_adaptiveclimbing.input.at(3),"inputFL_us");
	  addInspectableValue("input(4)",&control_adaptiveclimbing.input.at(4),"inputFR_us");
  }

//  if(plot_laserscanner)
//  {
//	  addInspectableValue("laserOutput",&control_adaptiveclimbing.laserOutput,"FR_us");
//  }

  if(plot_reversegear)
  {
    addInspectableValue("R",&control_adaptiveclimbing.input.at(3),"Rechts");
    addInspectableValue("L",&control_adaptiveclimbing.input.at(4),"Links");

  }
  if(plot_testbjc)
  {
    addInspectableValue("BJ_OUTPUT0",&control_adaptiveclimbing.bj_output.at(0),"BJ_OUTPUT");
    addInspectableValue("BJ_OUTPUT1",&control_adaptiveclimbing.bj_output.at(1),"BJ_OUTPUT");
    addInspectableValue("BJ_OUTPUT2",&control_adaptiveclimbing.bj_output.at(2),"BJ_OUTPUT");
    addInspectableValue("BJ_OUTPUT3",&control_adaptiveclimbing.bj_output.at(3),"BJ_OUTPUT");
    addInspectableValue("BJ_OUTPUT4",&control_adaptiveclimbing.bj_output.at(4),"BJ_OUTPUT");
    addInspectableValue("BJ_OUTPUT5",&control_adaptiveclimbing.bj_output.at(5),"BJ_OUTPUT");
    addInspectableValue("BJ_activity0",&control_adaptiveclimbing.bj_activity.at(0),"BJ_OUTPUT");
    addInspectableValue("BJ_activity1",&control_adaptiveclimbing.bj_activity.at(1),"BJ_OUTPUT");
    addInspectableValue("BJ_activity2",&control_adaptiveclimbing.bj_activity.at(2),"BJ_OUTPUT");
    addInspectableValue("BJ_activity3",&control_adaptiveclimbing.bj_activity.at(3),"BJ_OUTPUT");
    addInspectableValue("BJ_activity4",&control_adaptiveclimbing.bj_activity.at(4),"BJ_OUTPUT");
    addInspectableValue("BJ_activity5",&control_adaptiveclimbing.bj_activity.at(5),"BJ_OUTPUT");
    addInspectableValue("BJ_TEST0",&preprocessing_learning.test_output.at(0),"BJ_OUTPUT");
    addInspectableValue("BJ_TEST1",&preprocessing_learning.test_output.at(1),"BJ_OUTPUT");
    addInspectableValue("BJ_TEST2",&preprocessing_learning.test_output.at(2),"BJ_OUTPUT");
    addInspectableValue("BJ_TEST3",&preprocessing_learning.test_output.at(3),"BJ_OUTPUT");
    addInspectableValue("BJ_TEST4",&preprocessing_learning.test_output.at(4),"BJ_OUTPUT");
  }

  if(plot_testlaser)
  {
	  addInspectableValue("BJ_OUTPUT0",&control_adaptiveclimbing.bj_output.at(0),"BJ_OUTPUT");

  }

  //Add edit parameter on terminal
  Configurable::addParameter("cin", &control_adaptiveclimbing.Control_input,  /*minBound*/ -10,  /*maxBound*/ 10,
      "test description" );
  Configurable::addParameter("input3", &control_adaptiveclimbing.input.at(3),  /*minBound*/ -1,  /*maxBound*/ 1,
      "test description" );
  Configurable::addParameter("input4", &control_adaptiveclimbing.input.at(4),  /*minBound*/ -1,  /*maxBound*/ 1,
      "test description" );
  // prepare name;
  Configurable::insertCVSInfo(name, "$RCSfile: amosIIcontrol.cpp,v $",
      "$Revision: 0.1 $");


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


  for (unsigned int i = 0; i < AMOSII_SENSOR_MAX; i++) {
    x.at(i) = x_[i];
  }

  //1) Neural preprocessing and learning------------


  std::vector <vector<double> > x_prep = preprocessing_learning.step_npp(x);



  //2) Neural locomotion control------


     y = control_adaptiveclimbing.step_nlc(x, x_prep,/*Footinhibition = false, true*/false);





  /***Don't touch****Set Motor outputs begin *******************/

  for (unsigned int i = 0; i < AMOSII_MOTOR_MAX; i++) {
    y_[i] = y.at(i);
  }

  /*******Set  Motor outputs end *******************/

  // update step counter
  t++;
}
;

/** stores the controller values to a given file. */
bool AmosIIControl::store(FILE* f) const {
  return true;
}

/** loads the controller values from a given file. */
bool AmosIIControl::restore(FILE* f) {
  //	Configurable::parse(f);
  return true;
}

