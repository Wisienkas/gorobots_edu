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
//#include <ode_robots/amosII.h>
using namespace matrix;
using namespace std;

AmosIIControl::AmosIIControl() :
  AbstractController("AmosIIControl", "$Id: amosIIcontrol.cpp,v 0.1 $") {

  //---ADD YOUR initialization here---//

  t = 0; // step counter



  //---PLOT OPTIONS---//

  testing = false;
  plot_irlearning = true;   /*Plot IR learning for experiments or testing*/
  plot_irsignals = false;   /*Plot IR signals for experiments or testing*/
  plot_preprofc = false;    /*Plot preprocessed foot contact sensor signals for testing*/
  plot_preproirs = false;    /*Plot preprocessed IR sensor signals for testing*/
  plot_preproirleg = false; /*Plot preprocessed IR leg sensor signals for testing*/
  plot_fmodel_ctr = false;  /*Plot fmodel signals of ctr joints for testing*/
  plot_reflex_fs = false;   /*Plot fs signals used for reflexes*/
  plot_post_ctr = false;    /*Plot postprocessed ctr signals for testing*/
  plot_fmodel_errors = false; /*Plot errors of fmodel*/
  plot_fmodel_w = false;    /*Plot weights of fmodel*/
  plot_reversegear = false;  /*Plot obstacle avoidance behavior of robot*/
  plot_nlc = false;       /*Plot nlc control signals*/
  plot_testbjc = false;    /*Plot BJC for testing*/
  plot_cpg = false;    /*Plot CPG for testing*/
  plot_fmodel_counter = false;

  //Call this function with your changeable parameters here//

  //Changeable to terminal
  // addParameter("WeightH1_H1", &control_your_extension.WeightH1_H1);


  addInspectableValue("ac_motor",&control_adaptiveclimbing.ac_motor,"ac_motor");
  addInspectableValue("i_motor",&control_adaptiveclimbing.i_motor,"i_motor");
  addInspectableValue("motor_power_con",&control_adaptiveclimbing.motor_power_con,"motor_power_con");

  addInspectableValue("inclino_x",&control_adaptiveclimbing.incli_x,"incli_x");
  addInspectableValue("inclino_y",&control_adaptiveclimbing.incli_y,"incli_y");

  addInspectableValue("BX",&preprocessing_learning.preprosensor.at(BX_pos),"incli_x");
  addInspectableValue("speedX",&preprocessing_learning.xspeed,"incli_x");
  addInspectableValue("speedX",&preprocessing_learning.c,"incli_x");


  //1) Neural preprocessing plotting-----

  if(testing)
  {
    addInspectableValue("CPGtest0",&control_adaptiveclimbing.cpg_output.at(0),"CPGtest0");
    addInspectableValue("CPGtest1",&control_adaptiveclimbing.cpg_output.at(1),"CPGtest1");
//    addInspectableValue("pCPGtest0",&control_adaptiveclimbing.test_pcpg_output.at(0),"pCPGtest0");
//    addInspectableValue("pCPGtest1",&control_adaptiveclimbing.test_pcpg_output.at(1),"pCPGtest1");
    addInspectableValue("PSN0",&control_adaptiveclimbing.psn_output.at(10),"PSN0");
    addInspectableValue("PSN1",&control_adaptiveclimbing.psn_output.at(11),"PSN1");
    addInspectableValue("VRN0",&control_adaptiveclimbing.vrn_output.at(12),"VRN0");
    addInspectableValue("VRN1",&control_adaptiveclimbing.vrn_output.at(13),"VRN1");
//    addInspectableValue("CPGact0", &control_adaptiveclimbing.cpg_act_0, "CPGtest0");
//    addInspectableValue("CPGact1", &control_adaptiveclimbing.cpg_act_1, "CPGtest1");
//    addInspectableValue("CPGw10", &control_adaptiveclimbing.cpg_w_0, "CPGtest0");
//    addInspectableValue("CPGw01", &control_adaptiveclimbing.cpg_w_1, "CPGtest1");
//    addInspectableValue("CPGrec0", &control_adaptiveclimbing.cpg_rec_0, "CPGtest0");
//    addInspectableValue("CPGrec1", &control_adaptiveclimbing.cpg_rec_1, "CPGtest1");
//    addInspectableValue("CPGbias0", &control_adaptiveclimbing.cpg_bias_0, "CPGtest0");
//    addInspectableValue("CPGbias1", &control_adaptiveclimbing.cpg_bias_1, "CPGtest1");

  }

  if(plot_preprofc)
  {
    addInspectableValue("R0_fs",&preprocessing_learning.preprosensor.at(R0_fs),"R0_fs");
    addInspectableValue("R1_fs",&preprocessing_learning.preprosensor.at(R1_fs),"R1_fs");
    addInspectableValue("R2_fs",&preprocessing_learning.preprosensor.at(R2_fs),"R2_fs");
    addInspectableValue("L0_fs",&preprocessing_learning.preprosensor.at(L0_fs),"L0_fs");
    addInspectableValue("L1_fs",&preprocessing_learning.preprosensor.at(L1_fs),"L1_fs");
    addInspectableValue("L2_fs",&preprocessing_learning.preprosensor.at(L2_fs),"L2_fs");
    addInspectableValue("BJ_as",&preprocessing_learning.preprosensor.at(BJ_as),"L1_fs");
  }
  if(plot_preproirs)
  {
    addInspectableValue("R0_irs",&preprocessing_learning.preprosensor.at(R0_irs),"R0_irs");
    addInspectableValue("R1_irs",&preprocessing_learning.preprosensor.at(R1_irs),"R1_irs");
    addInspectableValue("R2_irs",&preprocessing_learning.preprosensor.at(R2_irs),"R2_irs");
    addInspectableValue("L0_irs",&preprocessing_learning.preprosensor.at(L0_irs),"L0_irs");
    addInspectableValue("L1_irs",&preprocessing_learning.preprosensor.at(L1_irs),"L1_irs");
    addInspectableValue("L2_irs",&preprocessing_learning.preprosensor.at(L2_irs),"L2_irs");
  }
//  addInspectableValue("CR0_fs", &control_adaptiveclimbing.reflex_fs.at(R0_fs), "CL1_fs");
//  addInspectableValue("CR1_fs", &control_adaptiveclimbing.reflex_fs.at(R1_fs), "CL1_fs");
//  addInspectableValue("CR2_fs", &control_adaptiveclimbing.reflex_fs.at(R2_fs), "CL1_fs");
//  addInspectableValue("CL0_fs", &control_adaptiveclimbing.reflex_fs.at(L0_fs), "CL1_fs");
//  addInspectableValue("CL1_fs", &control_adaptiveclimbing.reflex_fs.at(L1_fs), "CL1_fs");
//  addInspectableValue("CL2_fs", &control_adaptiveclimbing.reflex_fs.at(L2_fs), "CL1_fs");
//
//  addInspectableValue("CR0_m", &control_adaptiveclimbing.m_pre.at(CR0_m), "CL1_fs");
//  addInspectableValue("CR1_m", &control_adaptiveclimbing.m_pre.at(CR1_m), "CL1_fs");
//  addInspectableValue("CR2_m", &control_adaptiveclimbing.m_pre.at(CR2_m), "CL1_fs");
//  addInspectableValue("CL0_m", &control_adaptiveclimbing.m_pre.at(CL0_m), "CL1_fs");
//  addInspectableValue("CL1_m", &control_adaptiveclimbing.m_pre.at(CL1_m), "CL1_fs");
//  addInspectableValue("CL2_m", &control_adaptiveclimbing.m_pre.at(CL2_m), "CL1_fs");
//
//  addInspectableValue("CR0_output", &control_adaptiveclimbing.fmodel.at(CR0_m)->output, "CL1_fs");
//  addInspectableValue("CR1_output", &control_adaptiveclimbing.fmodel.at(CR1_m)->output, "CL1_fs");
//  addInspectableValue("CR2_output", &control_adaptiveclimbing.fmodel.at(CR2_m)->output, "CL1_fs");
//  addInspectableValue("CL0_output", &control_adaptiveclimbing.fmodel.at(CL0_m)->output, "CL1_fs");
//  addInspectableValue("CL1_output", &control_adaptiveclimbing.fmodel.at(CL1_m)->output, "CL1_fs");
//  addInspectableValue("CL2_output", &control_adaptiveclimbing.fmodel.at(CL2_m)->output, "CL1_fs");
//
  addInspectableValue("CR0_outputfinal", &control_adaptiveclimbing.fmodel.at(CR0_m)->outputfinal, "CL1_fs");
  addInspectableValue("CR1_outputfinal", &control_adaptiveclimbing.fmodel.at(CR1_m)->outputfinal, "CL1_fs");
  addInspectableValue("CR2_outputfinal", &control_adaptiveclimbing.fmodel.at(CR2_m)->outputfinal, "CL1_fs");
  addInspectableValue("CL0_outputfinal", &control_adaptiveclimbing.fmodel.at(CL0_m)->outputfinal, "CL1_fs");
  addInspectableValue("CL1_outputfinal", &control_adaptiveclimbing.fmodel.at(CL1_m)->outputfinal, "CL1_fs");
  addInspectableValue("CL2_outputfinal", &control_adaptiveclimbing.fmodel.at(CL2_m)->outputfinal, "CL1_fs");
//
//  addInspectableValue("CR0_w", &control_adaptiveclimbing.fmodel.at(CR0_m)->input_w, "CL1_fs");
//  addInspectableValue("CR1_w", &control_adaptiveclimbing.fmodel.at(CR1_m)->input_w, "CL1_fs");
//  addInspectableValue("CR2_w", &control_adaptiveclimbing.fmodel.at(CR2_m)->input_w, "CL1_fs");
//  addInspectableValue("CL0_w", &control_adaptiveclimbing.fmodel.at(CL0_m)->input_w, "CL1_fs");
//  addInspectableValue("CL1_w", &control_adaptiveclimbing.fmodel.at(CL1_m)->input_w, "CL1_fs");
//  addInspectableValue("CL2_w", &control_adaptiveclimbing.fmodel.at(CL2_m)->input_w, "CL1_fs");
//
//  addInspectableValue("CR0_rec_w", &control_adaptiveclimbing.fmodel.at(CR0_m)->rec_w, "CL1_fs");
//  addInspectableValue("CR1_rec_w", &control_adaptiveclimbing.fmodel.at(CR1_m)->rec_w, "CL1_fs");
//  addInspectableValue("CR2_rec_w", &control_adaptiveclimbing.fmodel.at(CR2_m)->rec_w, "CL1_fs");
//  addInspectableValue("CL0_rec_w", &control_adaptiveclimbing.fmodel.at(CL0_m)->rec_w, "CL1_fs");
//  addInspectableValue("CL1_rec_w", &control_adaptiveclimbing.fmodel.at(CL1_m)->rec_w, "CL1_fs");
//  addInspectableVal//  addInspectableValue("CR0_error", &control_adaptiveclimbing.fmodel.at(CR0_m)->error_neg, "CL1_fs");
  //  addInspectableValue("CR1_error", &control_adaptiveclimbing.fmodel.at(CR1_m)->error_neg, "CL1_fs");
  //  addInspectableValue("CR2_error", &control_adaptiveclimbing.fmodel.at(CR2_m)->error_neg, "CL1_fs");
  //  addInspectableValue("CL0_error", &control_adaptiveclimbing.fmodel.at(CL0_m)->error_neg, "CL1_fs");
  //  addInspectableValue("CL1_error", &control_adaptiveclimbing.fmodel.at(CL1_m)->error_neg, "CL1_fs");
  //  addInspectableValue("CL2_error", &control_adaptiveclimbing.fmodel.at(CL2_m)->error_neg, "CL1_fs");
  //ue("CL2_rec_w", &control_adaptiveclimbing.fmodel.at(CL2_m)->rec_w, "CL1_fs");
//
//  addInspectableValue("CR0_bias", &control_adaptiveclimbing.fmodel.at(CR0_m)->bias, "CL1_fs");
//  addInspectableValue("CR1_bias", &control_adaptiveclimbing.fmodel.at(CR1_m)->bias, "CL1_fs");
//  addInspectableValue("CR2_bias", &control_adaptiveclimbing.fmodel.at(CR2_m)->bias, "CL1_fs");
//  addInspectableValue("CL0_bias", &control_adaptiveclimbing.fmodel.at(CL0_m)->bias, "CL1_fs");
//  addInspectableValue("CL1_bias", &control_adaptiveclimbing.fmodel.at(CL1_m)->bias, "CL1_fs");
//  addInspectableValue("CL2_bias", &control_adaptiveclimbing.fmodel.at(CL2_m)->bias, "CL1_fs");
//

  addInspectableValue("CR0_error", &control_adaptiveclimbing.fmodel.at(CR0_m)->error_neg, "CR0_fs");
  addInspectableValue("CR1_error", &control_adaptiveclimbing.fmodel.at(CR1_m)->error_neg, "CR1_fs");
  addInspectableValue("CR2_error", &control_adaptiveclimbing.fmodel.at(CR2_m)->error_neg, "CR2_fs");
  addInspectableValue("CL0_error", &control_adaptiveclimbing.fmodel.at(CL0_m)->error_neg, "CL0_fs");
  addInspectableValue("CL1_error", &control_adaptiveclimbing.fmodel.at(CL1_m)->error_neg, "CL1_fs");
  addInspectableValue("CL2_error", &control_adaptiveclimbing.fmodel.at(CL2_m)->error_neg, "CL2_fs");


//  addInspectableValue("CR0_errorW", &control_adaptiveclimbing.fmodel.at(CR0_m)->learning_error, "CL1_fs");
//  addInspectableValue("CR1_errorW", &control_adaptiveclimbing.fmodel.at(CR1_m)->learning_error, "CL1_fs");
//  addInspectableValue("CR2_errorW", &control_adaptiveclimbing.fmodel.at(CR2_m)->learning_error, "CL1_fs");
//  addInspectableValue("CL0_errorW", &control_adaptiveclimbing.fmodel.at(CL0_m)->learning_error, "CL1_fs");
//  addInspectableValue("CL1_errorW", &control_adaptiveclimbing.fmodel.at(CL1_m)->learning_error, "CL1_fs");
//  addInspectableValue("CL2_errorW", &control_adaptiveclimbing.fmodel.at(CL2_m)->learning_error, "CL1_fs");
//
//  addInspectableValue("CR0_lperror", &control_adaptiveclimbing.fmodel.at(CR0_m)->lowpass_error, "CL1_fs");
//  addInspectableValue("CR1_lperror", &control_adaptiveclimbing.fmodel.at(CR1_m)->lowpass_error, "CL1_fs");
//  addInspectableValue("CR2_lperror", &control_adaptiveclimbing.fmodel.at(CR2_m)->lowpass_error, "CL1_fs");
//  addInspectableValue("CL0_lperror", &control_adaptiveclimbing.fmodel.at(CL0_m)->lowpass_error, "CL1_fs");
//  addInspectableValue("CL1_lperror", &control_adaptiveclimbing.fmodel.at(CL1_m)->lowpass_error, "CL1_fs");
//  addInspectableValue("CL2_lperror", &control_adaptiveclimbing.fmodel.at(CL2_m)->lowpass_error, "CL1_fs");
//
//  addInspectableValue("CR0_acc_error", &control_adaptiveclimbing.fmodel.at(CR0_m)->acc_error, "CL1_fs");
//  addInspectableValue("CR1_acc_error", &control_adaptiveclimbing.fmodel.at(CR1_m)->acc_error, "CL1_fs");
//  addInspectableValue("CR2_acc_error", &control_adaptiveclimbing.fmodel.at(CR2_m)->acc_error, "CL1_fs");
//  addInspectableValue("CL0_acc_error", &control_adaptiveclimbing.fmodel.at(CL0_m)->acc_error, "CL1_fs");
//  addInspectableValue("CL1_acc_error", &control_adaptiveclimbing.fmodel.at(CL1_m)->acc_error, "CL1_fs");
//  addInspectableValue("CL2_acc_error", &control_adaptiveclimbing.fmodel.at(CL2_m)->acc_error, "CL1_fs");
  //2) Neural learning and memory plotting-----

  if(plot_irsignals)
  {
//    addInspectableValue("IR_Predic_R",&learningmemory_IRDistance.ir_predic.at(25),"IR_Predic_R");
//    addInspectableValue("IR_Reflex_R",&learningmemory_IRDistance.ir_reflex.at(25),"IR_Reflex_R");
//    addInspectableValue("IR_Predic_L",&learningmemory_IRDistance.ir_predic.at(26),"IR_Predic_L");
//    addInspectableValue("IR_Reflex_L",&learningmemory_IRDistance.ir_reflex.at(26),"IR_Reflex_L");
  }

  if(plot_irlearning)
    {
      addInspectableValue("IR_sensor_outR", &preprocessing_learning.ir_predic_activity.at(25), "IR_learn_rho1_R");
      addInspectableValue("IR_sensor_outL",&preprocessing_learning.ir_reflex_activity.at(25),"IR_learn_rho1_L");
      addInspectableValue("IR_Predic_output_R",&preprocessing_learning.ir_predic_output.at(25),"IR_Predic_output_R");
      addInspectableValue("IR_dReflex_output_R",&preprocessing_learning.ir_reflex_output.at(25),"IR_dReflex_output_R");
      addInspectableValue("IR_Sensor_output_L",&preprocessing_learning.ir_predic_output.at(26),"IR_Sensor_output_L");
      addInspectableValue("IR_dReflex_output_L",&preprocessing_learning.ir_reflex_output.at(26),"IR_dReflex_output_L");
      addInspectableValue("IR_learn_rho1_R",&preprocessing_learning.rho1.at(25),"IR_learn_rho1_R");
      addInspectableValue("IR_learn_rho1_L",&preprocessing_learning.rho1.at(26),"IR_learn_rho1_L");
      addInspectableValue("IR_learn_rho1rate_R",&preprocessing_learning.ir_learnrate.at(25),"IR_learnrate_rho1_R");
      addInspectableValue("IR_learn_rho1rate_L",&preprocessing_learning.ir_learnrate.at(26),"IR_learnrate_rho1_L");
      addInspectableValue("Relative_pos",&control_adaptiveclimbing.relative_position,"Relative_pos");
      addInspectableValue("Sum_Robot_speed",&control_adaptiveclimbing.sum_robot_speed,"Sum_Robot_speed");
      addInspectableValue("Average_Robot_speed",&control_adaptiveclimbing.average_robot_speed,"Average_Robot_speed");
      addInspectableValue("IR_learnoutput_Rdyn_lowpass",&preprocessing_learning.dyn_lowpass.at(25),"IR_learnoutput_R");
      addInspectableValue("IR_learnoutput_Ldyn_lowpass",&preprocessing_learning.dyn_lowpass.at(26),"IR_learnoutput_L");
      addInspectableValue("IR_learnoutput_R_prolong",&preprocessing_learning.irlearn_output_prolong.at(25),"IR_learnoutput_R");
      addInspectableValue("IR_learnoutput_L_prolong",&preprocessing_learning.irlearn_output_prolong.at(26),"IR_learnoutput_L");
      addInspectableValue("IR_learnoutput_R_delay",&preprocessing_learning.preprosensor.at(FR_us),"IR_learnoutput_R");
      addInspectableValue("IR_learnoutput_L_delay",&preprocessing_learning.preprosensor.at(FL_us),"IR_learnoutput_L");
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
//    addInspectableValue("postCR0",&control_adaptiveclimbing.fmodel_outputfinal.at(CR0_m),"postCR0");
//    addInspectableValue("postCR1",&control_adaptiveclimbing.fmodel_outputfinal.at(CR1_m),"postCR1");
//    addInspectableValue("postCR2",&control_adaptiveclimbing.fmodel_outputfinal.at(CR2_m),"postCR2");
//    addInspectableValue("postCL0",&control_adaptiveclimbing.fmodel_outputfinal.at(CL0_m),"postCL0");
//    addInspectableValue("postCL1",&control_adaptiveclimbing.fmodel_outputfinal.at(CL1_m),"postCL1");
//    addInspectableValue("postCL2",&control_adaptiveclimbing.fmodel_outputfinal.at(CL2_m),"postCL2");
  }
  if(plot_fmodel_errors)
  {
    addInspectableValue("errorR0_fs",&control_adaptiveclimbing.fmodel_error.at(CR0_m),"errorR0_fs");
    addInspectableValue("errorR1_fs",&control_adaptiveclimbing.fmodel_error.at(CR1_m),"errorR1_fs");
    addInspectableValue("errorR2_fs",&control_adaptiveclimbing.fmodel_error.at(CR2_m),"errorR2_fs");
    addInspectableValue("errorL0_fs",&control_adaptiveclimbing.fmodel_error.at(CL0_m),"errorL0_fs");
    addInspectableValue("errorL1_fs",&control_adaptiveclimbing.fmodel_error.at(CL1_m),"errorL1_fs");
    addInspectableValue("errorL2_fs",&control_adaptiveclimbing.fmodel_error.at(CL2_m),"errorL2_fs");
    addInspectableValue("errorR0_TC",&control_adaptiveclimbing.TC_error.at(TR0_m),"errorR0_TC");
    addInspectableValue("errorL0_TC",&control_adaptiveclimbing.TC_error.at(TL0_m),"errorL0_TC");
    addInspectableValue("errorR1_TC",&control_adaptiveclimbing.TC_error.at(TR1_m),"errorR1_TC");
    addInspectableValue("errorL1_TC",&control_adaptiveclimbing.TC_error.at(TL1_m),"errorL1_TC");
    addInspectableValue("errorR2_TC",&control_adaptiveclimbing.TC_error.at(TR2_m),"errorR2_TC");
    addInspectableValue("errorL2_TC",&control_adaptiveclimbing.TC_error.at(TL2_m),"errorL2_TC");

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
//  addInspectableValue("TCR_buffer",&control_adaptiveclimbing.tr_output.at(2),"BJ_OUTPUT");
//  addInspectableValue("FTI_buffer",&control_adaptiveclimbing.fr_output.at(2),"BJ_OUTPUT");
//  addInspectableValue("FR0_motorpre",&control_adaptiveclimbing.m_pre.at(FR0_m),"BJ_OUTPUT");
//  addInspectableValue("FR1_motorpre",&control_adaptiveclimbing.m_pre.at(FR1_m),"BJ_OUTPUT");
//  addInspectableValue("FR2_motorpre",&control_adaptiveclimbing.m_pre.at(FR2_m),"BJ_OUTPUT");
//  addInspectableValue("CR0",&control_adaptiveclimbing.postcr.at(CR0_m),"CR2");
//  addInspectableValue("CR0_old",&control_adaptiveclimbing.postcrold.at(CR0_m),"CR2_old");
//  addInspectableValue("CR0_motorpre",&control_adaptiveclimbing.m_pre.at(CR0_m),"BJ_OUTPUT");
//  addInspectableValue("CR1",&control_adaptiveclimbing.postcr.at(CR1_m),"CR2");
//  addInspectableValue("CR1_old",&control_adaptiveclimbing.postcrold.at(CR1_m),"CR2_old");
//  addInspectableValue("CR1_motorpre",&control_adaptiveclimbing.m_pre.at(CR1_m),"BJ_OUTPUT");
//  addInspectableValue("CR2",&control_adaptiveclimbing.postcr.at(CR2_m),"CR2");
//  addInspectableValue("CR2_old",&control_adaptiveclimbing.postcrold.at(CR2_m),"CR2_old");
//  addInspectableValue("CR2_motorpre",&control_adaptiveclimbing.m_pre.at(CR2_m),"BJ_OUTPUT");
//  addInspectableValue("CL0_motorpre",&control_adaptiveclimbing.m_pre.at(CL0_m),"BJ_OUTPUT");
//  addInspectableValue("CL1_motorpre",&control_adaptiveclimbing.m_pre.at(CL1_m),"BJ_OUTPUT");
//  addInspectableValue("CL2_motorpre",&control_adaptiveclimbing.m_pre.at(CL2_m),"BJ_OUTPUT");

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
//  addInspectableValue("T",&control_adaptiveclimbing.m_reflex.at(TL0_m),"BJ_OUTPUT");
//  addInspectableValue("C",&control_adaptiveclimbing.m_reflex.at(CL0_m),"BJ_OUTPUT");
//  addInspectableValue("F",&control_adaptiveclimbing.m_reflex.at(FL0_m),"BJ_OUTPUT");
////  addInspectableValue("T2",&control_adaptiveclimbing.m_reflex_t.at(TL0_m),"BJ_OUTPUT");
////  addInspectableValue("C2",&control_adaptiveclimbing.m_reflex_t.at(CL0_m),"BJ_OUTPUT");
////  addInspectableValue("F2",&control_adaptiveclimbing.m_reflex_t.at(FL0_m),"BJ_OUTPUT");
//  addInspectableValue("Tdeg",&control_adaptiveclimbing.m_deg.at(TL0_m),"BJ_OUTPUT");
//    addInspectableValue("Cdeg",&control_adaptiveclimbing.m_deg.at(CL0_m),"BJ_OUTPUT");
//    addInspectableValue("Fdeg",&control_adaptiveclimbing.m_deg.at(FL0_m),"BJ_OUTPUT");
////    addInspectableValue("T2deg",&control_adaptiveclimbing.m_deg_t.at(TL0_m),"BJ_OUTPUT");
////    addInspectableValue("C2deg",&control_adaptiveclimbing.m_deg_t.at(CL0_m),"BJ_OUTPUT");
////    addInspectableValue("F2deg",&control_adaptiveclimbing.m_deg_t.at(FL0_m),"BJ_OUTPUT");
////  addInspectableValue("dELTAm",&control_adaptiveclimbing.delta_m_reflex.at(TR0_m),"BJ_OUTPUT");
//  addInspectableValue("VRNR",&control_adaptiveclimbing.vrn_output.at(13),"");
//  addInspectableValue("VRNL",&control_adaptiveclimbing.vrn_output.at(12),"BJ_OUTPUT");
//  addInspectableValue("TCR",&control_adaptiveclimbing.tr_output.at(2),"");
//  addInspectableValue("TCL",&control_adaptiveclimbing.tl_output.at(2),"BJ_OUTPUT");
//
//  addInspectableValue("FTI",&control_adaptiveclimbing.fr_output.at(2),"BJ_OUTPUT");
//
//  addInspectableValue("TC0",&control_adaptiveclimbing.test_motor_activity.at(0),"TC0_MOTOR_ACTIVITY");



  //Add edit parameter on terminal
  Configurable::addParameter("cin", &control_adaptiveclimbing.nlc->Control_input,  /*minBound*/ -10,  /*maxBound*/ 10,
      "test description" );
  Configurable::addParameter("input3", &control_adaptiveclimbing.input.at(3),  /*minBound*/ -1,  /*maxBound*/ 1,
      "test description" );
  Configurable::addParameter("input4", &control_adaptiveclimbing.input.at(4),  /*minBound*/ -1,  /*maxBound*/ 1,
      "test description" );
  Configurable::addParameter("muR", &preprocessing_learning.ir_learnrate.at(25),  /*minBound*/ -10,  /*maxBound*/ 10,
      "test description" );
  Configurable::addParameter("muL", &preprocessing_learning.ir_learnrate.at(26),  /*minBound*/ -10,  /*maxBound*/ 10,
      "test description" );
  Configurable::addParameter("delay", &preprocessing_learning.delay,  /*minBound*/ 0,  /*maxBound*/ 500,
       "test description" );

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


  std::vector<double> x_prep = preprocessing_learning.step_npp(x);


//  //2) Neural learning and memory-----
//
//
//  std::vector<double> memory_out = learningmemory_IRDistance.step_nlm(x);


  //3) Neural locomotion control------


     y = control_adaptiveclimbing.step_nlc(x, x_prep,/*Footinhibition = false, true*/false);

  /*************************************/

  //4) Motor postprocessing/scaling   ----------------


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

