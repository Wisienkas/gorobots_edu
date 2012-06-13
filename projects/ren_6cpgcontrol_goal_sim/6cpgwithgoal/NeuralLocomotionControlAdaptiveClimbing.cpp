/*
 * NeuralLocomotionControlAdaptiveClimbing.cpp
 *
 *  Created on: May 2, 2011
 *      Author: poramate
 */

#include "NeuralLocomotionControlAdaptiveClimbing.h"

//initialize object by designed Ren Guanjiao
#include "ChaoscontrAndPostproc.h"
ChaoscontrAndPostproc ChaosCont(4,5,6,4,5,6);

//3) Step function of Neural locomotion control------

NeuralLocomotionControlAdaptiveClimbing::NeuralLocomotionControlAdaptiveClimbing(){

 		  //Save files
 		  //outFilenlc1.open("TC-joint.txt");
 		  //outFilenlcr.open("CTr-joint.txt");
		  //outFilenlft.open("FTi-joint.txt");

/*******************************************************************************
* Set vector size
*******************************************************************************/
 		  //---Set vector size----//

 		  //Input vector size
 		  input.resize(5);

 		  //CPG vector sizes
 		  cpg_activity.resize(2);
 		  cpg_output.resize(2);

 		  cpg_w.resize(2);
 		  for(unsigned int i=0; i<cpg_w.size(); i++)
 		  {
 			  cpg_w.at(i).resize(2);
 		  }
 		  
 		  //set Master or Client
 		  ChaosCont.memory = 10000;
 		  ChaosCont.memory1 = 10000;
 		  ChaosCont.memory2 = 10000;
 		  ChaosCont.memory3 = 10000;
 		  ChaosCont.memory4 = 10000;
 		  ChaosCont.synflag = true;
 		  ChaosCont.synflag1 = true;
 		  ChaosCont.synflag2 = true;
 		  ChaosCont.synflag3 = true;
 		  ChaosCont.synflag4 = true;
 		
		  //Neu1&2 by guanjiao
		  cpg_output1.resize(2);
		  cpg_output2.resize(2);
		  cpg_output3.resize(2);
		  cpg_output4.resize(2);
		  cpg_output5.resize(2);
		  
 		  //pCPG vector sizes
 		  /*pcpg_output.resize(2);
 		  pcpg_step.resize(2);
 		  set.resize(2);
 		  setold.resize(2);
 		  diffset.resize(2);
 		  countup.resize(2);
 		  countupold.resize(2);
 		  countdown.resize(2);
 		  countdownold.resize(2);
 		  deltaxup.resize(2);
 		  deltaxdown.resize(2);
 		  xup.resize(2);
 		  xdown.resize(2);
 		  yup.resize(2);
 		  ydown.resize(2);
 		  
 		  //Neu1 by guanjiao
 		  pcpg_output1.resize(2);
 		  pcpg_step1.resize(2);
 		  set1.resize(2);
 		  setold1.resize(2);
 		  diffset1.resize(2);
 		  countup1.resize(2);
 		  countupold1.resize(2);
 		  countdown1.resize(2);
 		  countdownold1.resize(2);
 		  deltaxup1.resize(2);
 		  deltaxdown1.resize(2);
 		  xup1.resize(2);
 		  xdown1.resize(2);
 		  yup1.resize(2);
 		  ydown1.resize(2);
 		  
 		  //Neu2 by guanjiao
 		  pcpg_output2.resize(2);
 		  pcpg_step2.resize(2);
 		  set2.resize(2);
 		  setold2.resize(2);
 		  diffset2.resize(2);
 		  countup2.resize(2);
 		  countupold2.resize(2);
 		  countdown2.resize(2);
 		  countdownold2.resize(2);
 		  deltaxup2.resize(2);
 		  deltaxdown2.resize(2);
 		  xup2.resize(2);
 		  xdown2.resize(2);
 		  yup2.resize(2);
 		  ydown2.resize(2);
 		  */

 		  //PSN vector sizes
 		  psn_activity.resize(12);
 		  psn_output.resize(12);

 		  psn_w.resize(12);
 		  for(unsigned int i=0; i<psn_w.size(); i++)
 		  {
 			  psn_w.at(i).resize(12);
 		  }
 		  psn_bias.resize(3);
 		  
 		  //PSN1 vector sizes --guanjiao
 		  psn_activity1.resize(12);
 		  psn_output1.resize(12);

 		  psn_w1.resize(12);
 		  for(unsigned int i=0; i<psn_w1.size(); i++)
 		  {
 			  psn_w1.at(i).resize(12);
 		  }
 		  psn_bias1.resize(3);
 		  
 		  //PSN2 vector sizes --guanjiao
 		  psn_activity2.resize(12);
 		  psn_output2.resize(12);

 		  psn_w2.resize(12);
 		  for(unsigned int i=0; i<psn_w2.size(); i++)
 		  {
 			  psn_w2.at(i).resize(12);
 		  }
 		  psn_bias2.resize(3);
 		  
 		  //PSN3 vector sizes --guanjiao
 		  psn_activity3.resize(12);
 		  psn_output3.resize(12);

 		  psn_w3.resize(12);
 		  for(unsigned int i=0; i<psn_w3.size(); i++)
 		  {
 			  psn_w3.at(i).resize(12);
 		  }
 		  psn_bias3.resize(3);
 		  
 		  //PSN4 vector sizes --guanjiao
 		  psn_activity4.resize(12);
 		  psn_output4.resize(12);

 		  psn_w4.resize(12);
 		  for(unsigned int i=0; i<psn_w4.size(); i++)
 		  {
 			  psn_w4.at(i).resize(12);
 		  }
 		  psn_bias4.resize(3);
 		  
 		  //PSN5 vector sizes --guanjiao
 		  psn_activity5.resize(12);
 		  psn_output5.resize(12);

 		  psn_w5.resize(12);
 		  for(unsigned int i=0; i<psn_w5.size(); i++)
 		  {
 			  psn_w5.at(i).resize(12);
 		  }
 		  psn_bias5.resize(3);

 		  //VRN vector sizes
 		  vrn_activity.resize(14);
 		  vrn_output.resize(14);

 		  vrn_w.resize(14);
 		  for(unsigned int i=0; i<vrn_w.size(); i++)
 		  {
 			  vrn_w.at(i).resize(14);
 		  }
 		  
 		  //VRN1 vector sizes --guanjiao
 		  vrn_activity1.resize(14);
 		  vrn_output1.resize(14);

 		  vrn_w1.resize(14);
 		  for(unsigned int i=0; i<vrn_w1.size(); i++)
 		  {
 			  vrn_w1.at(i).resize(14);
 		  }
 		  
 		  //VRN2 vector sizes --guanjiao
 		  vrn_activity2.resize(14);
 		  vrn_output2.resize(14);

 		  vrn_w2.resize(14);
 		  for(unsigned int i=0; i<vrn_w2.size(); i++)
 		  {
 			  vrn_w2.at(i).resize(14);
 		  }

 		  //Interconnections vector sizes
 		  psn_pcpg_w.resize(14);
 		  for(unsigned int i=0; i<psn_pcpg_w.size(); i++)
 		  {
 			  psn_pcpg_w.at(i).resize(14);
 		  }

 		  vrn_psn_w.resize(14);
 		  for(unsigned int i=0; i<vrn_psn_w.size(); i++)
 		  {
 			  vrn_psn_w.at(i).resize(14);
 		  }

 		  psn_input2_w.resize(2);
		  for(unsigned int i=0; i<psn_input2_w.size(); i++)
		  {
			  psn_input2_w.at(i).resize(2);
		  }
		  
		  //Interconnections vector sizes --guanjiao
 		  psn_pcpg_w1.resize(14);
 		  for(unsigned int i=0; i<psn_pcpg_w1.size(); i++)
 		  {
 			  psn_pcpg_w1.at(i).resize(14);
 		  }

 		  vrn_psn_w1.resize(14);
 		  for(unsigned int i=0; i<vrn_psn_w1.size(); i++)
 		  {
 			  vrn_psn_w1.at(i).resize(14);
 		  }

 		  psn_input2_w1.resize(2);
		  for(unsigned int i=0; i<psn_input2_w1.size(); i++)
		  {
			  psn_input2_w1.at(i).resize(2);
		  }
		  
		  //Interconnections vector sizes --guanjiao
 		  psn_pcpg_w2.resize(14);
 		  for(unsigned int i=0; i<psn_pcpg_w2.size(); i++)
 		  {
 			  psn_pcpg_w2.at(i).resize(14);
 		  }

 		  vrn_psn_w2.resize(14);
 		  for(unsigned int i=0; i<vrn_psn_w2.size(); i++)
 		  {
 			  vrn_psn_w2.at(i).resize(14);
 		  }

 		  psn_input2_w2.resize(2);
		  for(unsigned int i=0; i<psn_input2_w2.size(); i++)
		  {
			  psn_input2_w2.at(i).resize(2);
		  }
		  
		  //Interconnections vector sizes --guanjiao
 		  psn_pcpg_w3.resize(14);
 		  for(unsigned int i=0; i<psn_pcpg_w3.size(); i++)
 		  {
 			  psn_pcpg_w3.at(i).resize(14);
 		  }

 		  /*vrn_psn_w2.resize(14);
 		  for(unsigned int i=0; i<vrn_psn_w2.size(); i++)
 		  {
 			  vrn_psn_w2.at(i).resize(14);
 		  }*/

 		  psn_input2_w3.resize(2);
		  for(unsigned int i=0; i<psn_input2_w3.size(); i++)
		  {
			  psn_input2_w3.at(i).resize(2);
		  }
		  
		  //Interconnections vector sizes --guanjiao
 		  psn_pcpg_w4.resize(14);
 		  for(unsigned int i=0; i<psn_pcpg_w4.size(); i++)
 		  {
 			  psn_pcpg_w4.at(i).resize(14);
 		  }

 		  /*vrn_psn_w2.resize(14);
 		  for(unsigned int i=0; i<vrn_psn_w2.size(); i++)
 		  {
 			  vrn_psn_w2.at(i).resize(14);
 		  }*/

 		  psn_input2_w4.resize(2);
		  for(unsigned int i=0; i<psn_input2_w4.size(); i++)
		  {
			  psn_input2_w4.at(i).resize(2);
		  }
		  
		  //Interconnections vector sizes --guanjiao
 		  psn_pcpg_w5.resize(14);
 		  for(unsigned int i=0; i<psn_pcpg_w5.size(); i++)
 		  {
 			  psn_pcpg_w5.at(i).resize(14);
 		  }

 		  /*vrn_psn_w2.resize(14);
 		  for(unsigned int i=0; i<vrn_psn_w2.size(); i++)
 		  {
 			  vrn_psn_w2.at(i).resize(14);
 		  }*/

 		  psn_input2_w5.resize(2);
		  for(unsigned int i=0; i<psn_input2_w5.size(); i++)
		  {
			  psn_input2_w5.at(i).resize(2);
		  }

		  tr_activity.resize(3);
		  tl_activity.resize(3);
		  cr_activity.resize(3);
		  cl_activity.resize(3);
		  fr_activity.resize(3);
		  fl_activity.resize(3);
		  bj_activity.resize(1);

		  tr_output.resize(3);
		  tl_output.resize(3);
		  cr_output.resize(3);
		  cl_output.resize(3);
		  fr_output.resize(3);
		  fl_output.resize(3);
		  bj_output.resize(1);

		  cr_outputold.resize(3);
		  cl_outputold.resize(3);
		  diffcr_output.resize(3);
		  diffcl_output.resize(3);
		  postcr.resize(3);
		  postcl.resize(3);
		  postcrold.resize(3);
		  postclold.resize(3);

		  buffer_t.resize(108);
		  buffer_c.resize(108);
		  buffer_f.resize(108);
		  buffer_tl.resize(108);
		  buffer_cl.resize(108);
		  buffer_fl.resize(108);
		  
		  buffer_t1.resize(108); //added by guanjiao
		  buffer_c1.resize(108);
		  buffer_f1.resize(108);
		  buffer_tl1.resize(108);
		  buffer_cl1.resize(108);
		  buffer_fl1.resize(108);
		  
		  buffer_t2.resize(108);
		  buffer_c2.resize(108);
		  buffer_f2.resize(108);
		  buffer_tl2.resize(108);
		  buffer_cl2.resize(108);
		  buffer_fl2.resize(108);


		  m_pre.resize(19);
		  m_reflex.resize(19);
 		  m.resize(19);
 		  m_deg.resize(19);


 		  //---Reflex motor neurons
 		  fmodel_cmr_activity.resize(3);
 		  fmodel_cmr_output.resize(3);
 		  fmodel_cmr_error.resize(3);
 		  fmodel_cml_activity.resize(3);
 		  fmodel_cml_output.resize(3);
 		  fmodel_cml_error.resize(3);

 		  fmodel_cmr_outputfinal.resize(3);
 		  fmodel_cml_outputfinal.resize(3);

 		  //---Reflex foot sensors
 		  reflex_R_fs.resize(3);
 		  reflex_L_fs.resize(3);

 		  //Learning forward models to expected foot sensors
 		  fmodel_cmr_w.resize(3);
 		  fmodel_fmodel_cmr_w.resize(3);
 		  fmodel_post_cmr_w.resize(3);
 		  fmodel_cmr_bias.resize(3);
 		  acc_cmr_error.resize(3);
 		  acc_cmr_error_old.resize(3);
 		  deri_acc_cmr_error.resize(3);
 		  acc_cmr_error_elev.resize(3);
 		  error_cmr_elev.resize(3);
 		  lr_fmodel_cr.resize(3);
 		  counter_cr.resize(3);

 		  fmodel_cml_w.resize(3);
 		  fmodel_fmodel_cml_w.resize(3);
 		  fmodel_post_cml_w.resize(3);
 		  fmodel_cml_bias.resize(3);
 		  acc_cml_error.resize(3);
 		  acc_cml_error_old.resize(3);
 		  deri_acc_cml_error.resize(3);
 		  acc_cml_error_elev.resize(3);
 		  error_cml_elev.resize(3);
 		  lr_fmodel_cl.resize(3);
 		  counter_cl.resize(3);

 		  lowpass_cmr_error_activity.resize(3);
 		  lowpass_cmr__error_output.resize(3);
 		  lowpass_cmr_w.resize(3);
 		  lowpass_lowpass_cmr_w.resize(3);
 		  lowpass_cmr_bias.resize(3);

 		  lowpass_cml_error_activity.resize(3);
 		  lowpass_cml__error_output.resize(3);
 		  lowpass_cml_w.resize(3);
 		  lowpass_lowpass_cml_w.resize(3);
 		  lowpass_cml_bias.resize(3);

 		  //Emd_2D another mechanism for forward models
 		  a1_r.resize(3);
 		  a2_r.resize(3);
 		  a3_r.resize(3);
 		  fcn_r.resize(3);
 		  fac_r.resize(3);
 		  pred_r.resize(3);
 		  normxsq_r.resize(3);
 		  a1_l.resize(3);
 		  a2_l.resize(3);
 		  a3_l.resize(3);
 		  fcn_l.resize(3);
 		  fac_l.resize(3);
 		  pred_l.resize(3);
 		  normxsq_l.resize(3);
 		  delay_CR0.resize(108);
 		  delay_CR1.resize(108);
 		  delay_CR2.resize(108);
 		  delay_CL0.resize(108);
 		  delay_CL1.resize(108);
 		  delay_CL2.resize(108);
 		  m_pre_delay.resize(19);


 		  //Motor ranges Min, Max
 		  min_ctr_nwalking_deg.resize(3);
 		  max_ctr_nwalking_deg.resize(3);
 		  min_ctr_nwalking.resize(3);
 		  max_ctr_nwalking.resize(3);
 		  offset_ctr.resize(3);

 		  min_ctl_nwalking_deg.resize(3);
 		  max_ctl_nwalking_deg.resize(3);
 		  min_ctl_nwalking.resize(3);
 		  max_ctl_nwalking.resize(3);
 		  offset_ctl.resize(3);

 		  min_ftir_nwalking_deg.resize(3);
 		  max_ftir_nwalking_deg.resize(3);
 		  min_ftir_nwalking.resize(3);
 		  max_ftir_nwalking.resize(3);
 		  offset_ftir.resize(3);

 		  min_ftil_nwalking_deg.resize(3);
 		  max_ftil_nwalking_deg.resize(3);
 		  min_ftil_nwalking.resize(3);
 		  max_ftil_nwalking.resize(3);
 		  offset_ftil.resize(3);


 		  //Using save text

 		  m_r0_t.resize(8000);
 		  m_r1_t.resize(8000);
 		  m_r2_t.resize(8000);
 		  m_l0_t.resize(8000);
 		  m_l1_t.resize(8000);
 		  m_l2_t.resize(8000);



/*******************************************************************************
*  Initial parameters
*******************************************************************************/
 		  //---Initial parameters----//
 		  //---initialize leg damage flag--
 		  LegDamage_flag[LEG_RF] = false;
 		  LegDamage_flag[LEG_RM] = false;
 		  LegDamage_flag[LEG_RH] = false;
 		  LegDamage_flag[LEG_LF] = false;
 		  LegDamage_flag[LEG_LM] = false;
 		  LegDamage_flag[LEG_LH] = false;

 		  //---Inputs

 		  input.at(0) = 1;
 		  input.at(1) = 1;
 		  input.at(2) = 1; // 0, or 1
 		  input.at(3) = 1;
 		  input.at(4) = 1;


		  //---CPG weights

 		  cpg_activity.at(0) = 0.1; // Initialization
 		  cpg_activity.at(1) = 0.1; // Initialization
 		  cpg_output.at(0) = 0.1; // Initialization
 		  cpg_output.at(1) = 0.1; // Initialization

 		  Control_input = 0.05;
		  cpg_w.at(0).at(0) =  1.5;
		  cpg_w.at(0).at(1) =  0.4;
		  cpg_w.at(1).at(0) =  -0.4;
		  cpg_w.at(1).at(1) =  1.5;
		  //network bias
		  cpg_bias =        0.0;//0.01;


 	      //---PSN weights

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
		  psn_bias.at(0) =  1;
		  psn_bias.at(1) =  0.5;
		  psn_bias.at(2) = -1.35;

 	      //---VRN weights

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


 	      //pCPG to PSN connection weights
 	      psn_pcpg_w.at(2).at(0) = 0.5;
 	      psn_pcpg_w.at(3).at(1) = 0.5;
 	      psn_pcpg_w.at(4).at(1) = 0.5;
 	      psn_pcpg_w.at(5).at(0) = 0.5;

 	      //PSN to VRN connection weights
 	      vrn_psn_w.at(0).at(11) = 1.75;
 	      vrn_psn_w.at(2).at(11) = 1.75;


 	      //input2 to PSN connection weights
 	      psn_input2_w.at(0).at(0) = -1.0;
 	      psn_input2_w.at(1).at(0) = 1.0;
 	      //input3 to VRN connection weight
 	      vrn_input3_w = 5;
 	      //input4 to VRN connection weight
 	      vrn_input4_w = 5;
 	      
 	      //---PSN1 weights ---GUANJIAO

 	      psn_w1.at(2).at(0) = -5.0;
 	      psn_w1.at(3).at(1) = -5.0;
 	      psn_w1.at(4).at(0) = -5.0;
 	      psn_w1.at(5).at(1) = -5.0;
 	      psn_w1.at(6).at(2) =  0.5;
 	      psn_w1.at(7).at(3) =  0.5;
 	      psn_w1.at(8).at(4) =  0.5;
 	      psn_w1.at(9).at(5) =  0.5;
 	      psn_w1.at(10).at(6) = 3.0;
 	      psn_w1.at(10).at(7) = 3.0;
 	      psn_w1.at(11).at(8) = 3.0;
 	      psn_w1.at(11).at(9) = 3.0;
 		  //network bias
		  psn_bias1.at(0) =  1;
		  psn_bias1.at(1) =  0.5;
		  psn_bias1.at(2) = -1.35;

 	      //---VRN1 weights --guanjiao

 	      vrn_w1.at(4).at(0) =    1.7246;
 	      vrn_w1.at(4).at(1) =    1.7246;
 	      vrn_w1.at(5).at(0) =   -1.7246;
 	      vrn_w1.at(5).at(1) =   -1.7246;
 	      vrn_w1.at(6).at(0) =    1.7246;
 	      vrn_w1.at(6).at(1) =   -1.7246;
 	      vrn_w1.at(7).at(0) =   -1.7246;
 	      vrn_w1.at(7).at(1) =    1.7246;
 	      vrn_w1.at(8).at(2) =    1.7246;
 	      vrn_w1.at(8).at(3) =    1.7246;
 	      vrn_w1.at(9).at(2) =   -1.7246;
 	      vrn_w1.at(9).at(3) =   -1.7246;
 	      vrn_w1.at(10).at(2) =   1.7246;
 	      vrn_w1.at(10).at(3) =  -1.7246;
 	      vrn_w1.at(11).at(2) =  -1.7246;
 	      vrn_w1.at(11).at(3) =   1.7246;
 	      vrn_w1.at(12).at(4) =   0.5;
 	      vrn_w1.at(12).at(5) =   0.5;
 	      vrn_w1.at(12).at(6) =  -0.5;
 	      vrn_w1.at(12).at(7) =  -0.5;
 	      vrn_w1.at(13).at(8) =   0.5;
 	      vrn_w1.at(13).at(9) =   0.5;
 	      vrn_w1.at(13).at(10) = -0.5;
 	      vrn_w1.at(13).at(11) = -0.5;
 	      //network bias
 	      vrn_bias1 =       -2.48285;


 	      //pCPG to PSN connection weights
 	      psn_pcpg_w1.at(2).at(0) = 0.5;
 	      psn_pcpg_w1.at(3).at(1) = 0.5;
 	      psn_pcpg_w1.at(4).at(1) = 0.5;
 	      psn_pcpg_w1.at(5).at(0) = 0.5;

 	      //PSN to VRN connection weights
 	      vrn_psn_w1.at(0).at(11) = 1.75;
 	      vrn_psn_w1.at(2).at(11) = 1.75;


 	      //input2 to PSN connection weights
 	      psn_input2_w1.at(0).at(0) = -1.0;
 	      psn_input2_w1.at(1).at(0) = 1.0;
 	      //input3 to VRN connection weight
 	      vrn_input3_w1 = 5;
 	      //input4 to VRN connection weight
 	      vrn_input4_w1 = 5;

		  //---PSN2 weights ---GUANJIAO

 	      psn_w2.at(2).at(0) = -5.0;
 	      psn_w2.at(3).at(1) = -5.0;
 	      psn_w2.at(4).at(0) = -5.0;
 	      psn_w2.at(5).at(1) = -5.0;
 	      psn_w2.at(6).at(2) =  0.5;
 	      psn_w2.at(7).at(3) =  0.5;
 	      psn_w2.at(8).at(4) =  0.5;
 	      psn_w2.at(9).at(5) =  0.5;
 	      psn_w2.at(10).at(6) = 3.0;
 	      psn_w2.at(10).at(7) = 3.0;
 	      psn_w2.at(11).at(8) = 3.0;
 	      psn_w2.at(11).at(9) = 3.0;
 		  //network bias
		  psn_bias2.at(0) =  1;
		  psn_bias2.at(1) =  0.5;
		  psn_bias2.at(2) = -1.35;

 	      //---VRN2 weights --guanjiao

 	      vrn_w2.at(4).at(0) =    1.7246;
 	      vrn_w2.at(4).at(1) =    1.7246;
 	      vrn_w2.at(5).at(0) =   -1.7246;
 	      vrn_w2.at(5).at(1) =   -1.7246;
 	      vrn_w2.at(6).at(0) =    1.7246;
 	      vrn_w2.at(6).at(1) =   -1.7246;
 	      vrn_w2.at(7).at(0) =   -1.7246;
 	      vrn_w2.at(7).at(1) =    1.7246;
 	      vrn_w2.at(8).at(2) =    1.7246;
 	      vrn_w2.at(8).at(3) =    1.7246;
 	      vrn_w2.at(9).at(2) =   -1.7246;
 	      vrn_w2.at(9).at(3) =   -1.7246;
 	      vrn_w2.at(10).at(2) =   1.7246;
 	      vrn_w2.at(10).at(3) =  -1.7246;
 	      vrn_w2.at(11).at(2) =  -1.7246;
 	      vrn_w2.at(11).at(3) =   1.7246;
 	      vrn_w2.at(12).at(4) =   0.5;
 	      vrn_w2.at(12).at(5) =   0.5;
 	      vrn_w2.at(12).at(6) =  -0.5;
 	      vrn_w2.at(12).at(7) =  -0.5;
 	      vrn_w2.at(13).at(8) =   0.5;
 	      vrn_w2.at(13).at(9) =   0.5;
 	      vrn_w2.at(13).at(10) = -0.5;
 	      vrn_w2.at(13).at(11) = -0.5;
 	      //network bias
 	      vrn_bias2 =       -2.48285;


 	      //pCPG to PSN connection weights
 	      psn_pcpg_w2.at(2).at(0) = 0.5;
 	      psn_pcpg_w2.at(3).at(1) = 0.5;
 	      psn_pcpg_w2.at(4).at(1) = 0.5;
 	      psn_pcpg_w2.at(5).at(0) = 0.5;

 	      //PSN to VRN connection weights
 	      vrn_psn_w2.at(0).at(11) = 1.75;
 	      vrn_psn_w2.at(2).at(11) = 1.75;


 	      //input2 to PSN connection weights
 	      psn_input2_w2.at(0).at(0) = -1.0;
 	      psn_input2_w2.at(1).at(0) = 1.0;
 	      //input3 to VRN connection weight
 	      vrn_input3_w2 = 5;
 	      //input4 to VRN connection weight
 	      vrn_input4_w2 = 5;
		
		  //---PSN3 weights ---GUANJIAO

 	      psn_w3.at(2).at(0) = -5.0;
 	      psn_w3.at(3).at(1) = -5.0;
 	      psn_w3.at(4).at(0) = -5.0;
 	      psn_w3.at(5).at(1) = -5.0;
 	      psn_w3.at(6).at(2) =  0.5;
 	      psn_w3.at(7).at(3) =  0.5;
 	      psn_w3.at(8).at(4) =  0.5;
 	      psn_w3.at(9).at(5) =  0.5;
 	      psn_w3.at(10).at(6) = 3.0;
 	      psn_w3.at(10).at(7) = 3.0;
 	      psn_w3.at(11).at(8) = 3.0;
 	      psn_w3.at(11).at(9) = 3.0;
 		  //network bias
		  psn_bias3.at(0) =  1;
		  psn_bias3.at(1) =  0.5;
		  psn_bias3.at(2) = -1.35;
		  
		  //pCPG to PSN connection weights
 	      psn_pcpg_w3.at(2).at(0) = 0.5;
 	      psn_pcpg_w3.at(3).at(1) = 0.5;
 	      psn_pcpg_w3.at(4).at(1) = 0.5;
 	      psn_pcpg_w3.at(5).at(0) = 0.5;
 	      
 	      //input2 to PSN connection weights
 	      psn_input2_w3.at(0).at(0) = -1.0;
 	      psn_input2_w3.at(1).at(0) = 1.0;
 	      
 	      //---PSN4 weights ---GUANJIAO

 	      psn_w4.at(2).at(0) = -5.0;
 	      psn_w4.at(3).at(1) = -5.0;
 	      psn_w4.at(4).at(0) = -5.0;
 	      psn_w4.at(5).at(1) = -5.0;
 	      psn_w4.at(6).at(2) =  0.5;
 	      psn_w4.at(7).at(3) =  0.5;
 	      psn_w4.at(8).at(4) =  0.5;
 	      psn_w4.at(9).at(5) =  0.5;
 	      psn_w4.at(10).at(6) = 3.0;
 	      psn_w4.at(10).at(7) = 3.0;
 	      psn_w4.at(11).at(8) = 3.0;
 	      psn_w4.at(11).at(9) = 3.0;
 		  //network bias
		  psn_bias4.at(0) =  1;
		  psn_bias4.at(1) =  0.5;
		  psn_bias4.at(2) = -1.35;
		  
		  //pCPG to PSN connection weights
 	      psn_pcpg_w4.at(2).at(0) = 0.5;
 	      psn_pcpg_w4.at(3).at(1) = 0.5;
 	      psn_pcpg_w4.at(4).at(1) = 0.5;
 	      psn_pcpg_w4.at(5).at(0) = 0.5;
 	      
 	      //input2 to PSN connection weights
 	      psn_input2_w4.at(0).at(0) = -1.0;
 	      psn_input2_w4.at(1).at(0) = 1.0;
 	      
 	      //---PSN5 weights ---GUANJIAO

 	      psn_w5.at(2).at(0) = -5.0;
 	      psn_w5.at(3).at(1) = -5.0;
 	      psn_w5.at(4).at(0) = -5.0;
 	      psn_w5.at(5).at(1) = -5.0;
 	      psn_w5.at(6).at(2) =  0.5;
 	      psn_w5.at(7).at(3) =  0.5;
 	      psn_w5.at(8).at(4) =  0.5;
 	      psn_w5.at(9).at(5) =  0.5;
 	      psn_w5.at(10).at(6) = 3.0;
 	      psn_w5.at(10).at(7) = 3.0;
 	      psn_w5.at(11).at(8) = 3.0;
 	      psn_w5.at(11).at(9) = 3.0;
 		  //network bias
		  psn_bias5.at(0) =  1;
		  psn_bias5.at(1) =  0.5;
		  psn_bias5.at(2) = -1.35;
		  
		  //pCPG to PSN connection weights
 	      psn_pcpg_w5.at(2).at(0) = 0.5;
 	      psn_pcpg_w5.at(3).at(1) = 0.5;
 	      psn_pcpg_w5.at(4).at(1) = 0.5;
 	      psn_pcpg_w5.at(5).at(0) = 0.5;
 	      
 	      //input2 to PSN connection weights
 	      psn_input2_w5.at(0).at(0) = -1.0;
 	      psn_input2_w5.at(1).at(0) = 1.0;

 	      //initial integrator values
 	      count1 = 0; // counter for each time step the pulse train is constant
 	      count2 = 0;
 	      T1 = -1; //period time counter
 	      T2 = -1;
 	      period1 = 0; //period number (even values means full period)
 	      period2 = 0;
 	      y1 = 0; //y-values of triangle function
 	      y2 = 0;

 	      //Coxa joints threshold
 	      threshold_c = -0.9;

 	      //motor time delay
 	      tau = 18;//16;
 	      tau_l = 90;//48;
 	      time = 0;

	      //motor time delay
// 	      tau = 16;
// 	      tau_l = 16;
// 	      time = 0;

 	      //Learning forward models to expected foot sensors

 	      for(unsigned int i=0; i<fmodel_cmr_w.size(); i++)
 	      {
 	    	  /*fmodel_cmr_w.at(i) = 1.77;//3.0;//2.0 (Ok, Cin =0.05);//3.0 (WORK perfect, Cin =0.05);//4.0 (CR1 not work)//5.0;
	    	  fmodel_fmodel_cmr_w.at(i) = 1.339;//0.0;
 	    	  fmodel_post_cmr_w.at(i) = 20.0;
 	    	  fmodel_cmr_bias.at(i) = 1.5;//1.2;//2.0;//1.2;//1.0;

 	    	  fmodel_cml_w.at(i) = 3.0;//4.0;
 	    	  fmodel_fmodel_cml_w.at(i) = 0.0;
 	    	  fmodel_post_cml_w.at(i) = 20.0;
 	    	  fmodel_cml_bias.at(i) = 1.2;//2.0;//2.0;//1.2;//1.0;
*/


 	    	// MUST! fmodel_cmr_w.at(i)>fmodel_cmr_bias.at(i)
 	    	  fmodel_cmr_w.at(i) = 1.77;//2.0;//1.77;//2.0;//1.77;
 	    	  fmodel_fmodel_cmr_w.at(i) = 0.0;
 	    	  fmodel_post_cmr_w.at(i) = 50.0;//20.0;
 	    	  fmodel_cmr_bias.at(i) = 1.3;//1.5;

 	    	  fmodel_cml_w.at(i) = 1.77;//2.0;//1.77;//2.0;//1.77;
 	    	  fmodel_fmodel_cml_w.at(i) = 0.0;
 	    	  fmodel_post_cml_w.at(i) = 50.0;//20.0;
 	    	  fmodel_cml_bias.at(i) = 1.3;//1.5;

 	    	  lr_fmodel_cr.at(i) = 0.01;//0.01;//0.05;//0.3; // 0.1, 0.5
 	    	  counter_cr.at(i) = 0;
 	    	  lr_fmodel_cl.at(i) = 0.01;//0.01;//0.05; // 0.1, 0.5
 	    	  counter_cl.at(i) = 0;



 	      }

 	      for(unsigned int i=0; i<lowpass_cmr_error_activity.size(); i++)
 	      {
 	    	  lowpass_cmr_error_activity.at(i) = 0.0;
 	    	  lowpass_cmr__error_output.at(i) = 0.0;
 	    	  lowpass_cmr_w.at(i) = 1.0;
 	    	  lowpass_lowpass_cmr_w.at(i) = 5.5;
 	    	  lowpass_cmr_bias.at(i) = -4.5;

 	    	  lowpass_cml_error_activity.at(i) = 0.0;
 	    	  lowpass_cml__error_output.at(i) = 0.0;
 	    	  lowpass_cml_w.at(i) = 1.0;
 	    	  lowpass_lowpass_cml_w.at(i) = 5.0;
 	    	  lowpass_cml_bias.at(i) = -4.5;//-0.1;
 	      }


 	      // Initialization of Emd parameters
	      for(unsigned int i=0; i<a1_r.size(); i++)
 	      {
			  a1_r.at(i) = 1000;
	 		  a2_r.at(i) = -5000;
	 		  a3_r.at(i) = 0.1;
			  a1_l.at(i) = 1000;
	 		  a2_l.at(i) = -5000;
	 		  a3_l.at(i) = 0.1;
 	      }

 	      //Motor mapping

 	      //TC_front
 	      //Fix
 	      min_tc = -0.91; // network output range
 	      max_tc = 0.91;// network output range
 	      //Adjust
 	      min_tc_f_nwalking_deg = 0; //deg ** MIN -70 deg
 	      max_tc_f_nwalking_deg = 50; //deg ** MAX +70 deg
 	      min_tc_f_nwalking = -1;
 	      max_tc_f_nwalking = 1;

 	      //TC_middle
 	      //Adjust
 	      min_tc_m_nwalking_deg = -30; //deg ** MIN -60 deg
 	      max_tc_m_nwalking_deg = 20;//10; //deg ** MAX +60 deg
 	      min_tc_m_nwalking = -1;
 	      max_tc_m_nwalking = 1;

 	      //TC_rear
 	      //Adjust
 	      min_tc_r_nwalking_deg = -60; //deg ** MIN -70 deg
 	      max_tc_r_nwalking_deg = -10; //deg ** MAX +70 deg
 	      min_tc_r_nwalking = -1;
 	      max_tc_r_nwalking = 1;


 	      //CTR
 	      //Fix
 	      min_ctr = -1; // network output range
 	      max_ctr = 0.96;// network output range

 	      //FTI
 	      //Fix
 	      min_fti = -1; // network output range
 	      max_fti = 1;// network output range

 	      for(unsigned int i=0; i< min_ctr_nwalking_deg.size(); i++)
 	      {
 	    	  //Parameter set up for normal walking on flat terrain
 	    	  min_ctr_nwalking_deg.at(i) = 45;//48;//45;//50 is good; //40deg ** MIN -70 deg
 	    	  max_ctr_nwalking_deg.at(i) = 75; //75deg ** MAX +70 deg
 	    	  min_ctr_nwalking.at(i) = -1;
 	    	  max_ctr_nwalking.at(i) =1;

 	    	  min_ctl_nwalking_deg.at(i) = 45;//48;//45;//50 is good; //deg ** MIN -70 deg
 	    	  max_ctl_nwalking_deg.at(i) = 75; //deg ** MAX +70 deg
 	    	  min_ctl_nwalking.at(i) = -1;
 	    	  max_ctl_nwalking.at(i) = 1;

 	    	  min_ftir_nwalking_deg.at(i) = -130; //deg ** MIN -130 deg
 	    	  max_ftir_nwalking_deg.at(i) = -120; //deg ** MAX -20 deg
 	    	  min_ftir_nwalking.at(i) = -1;
 	    	  max_ftir_nwalking.at(i) = 1;

 	    	  min_ftil_nwalking_deg.at(i) = -130;
 	    	  max_ftil_nwalking_deg.at(i) = -120;
 	    	  min_ftil_nwalking.at(i) = -1;
 	    	  max_ftil_nwalking.at(i) = 1;
 	      }


 	      max_c = 115.0; // max range
 	      max_f = 110.0; // max range
 	      //if  max_c_offset, max_f_offset very low = 45 then robot will extend its legs very fast--> walk very high good for rough terrain
 	      //if  max_c_offset, max_f_offset very large = 180 then robot will extend its legs very slow--> walk very low good for normal walking
 	      max_c_offset = 110;//60/*Use to compare*/;//80;//100.0;//55.0; // Adjust from 45,...185=> 185 (cin= 0.02, wave), 45 (cin= 0.18, tripod)
 	      max_f_offset = 110;//60/*Use to compare*/;//80;//100.0;//55.0; // Adjust from 45,...185=> 185 (cin= 0.02, wave), 45 (cin= 0.18, tripod)
 	      //80,100,185 == amlost the same height when using forward model and foot but different height using only foot signal

 	      //
 	      //BJ
 	      //Fix
 	      min_bj = -1; // network output range
 	      max_bj = 1;	// network output range

 	      //Adjust
 	      min_bj_fwalking_deg = -45; //deg ** MIN -45 deg
 	      max_bj_fwalking_deg = 45; //deg ** MAX 45 deg

 	     //Other parameters
 	      global_count = 0;
 	      allfoot_off_ground = 0;

 	      //Test walking behavior with saved motor text
 	     initialized = false;
 	     ii = 0;

 	     m_r0_t_old = 0.0;
 	     m_r1_t_old = 0.0;
 	     m_r2_t_old = 0.0;
 	     m_l0_t_old = 0.0;
 	     m_l1_t_old = 0.0;
 	     m_l2_t_old = 0.0;

 	     alpha = 0;
 	     alpha_old = 0;
 	     alpha_tmp = 0;
 	     delta_alpha = 0;
 	     Deviation_of_y = 0;

/*******************************************************************************
*  CONTROL OPTION!!!!
*******************************************************************************/
 	      //Wiring connection
 	      option_wiring = 2; //1==Tripod, 2==Delayed line)

 	      //Selecting forward models
 	      option_fmodel = 1;
 	      //1==with threshold after fmodel & NO lowpass neuron after error;
 	      //2 == without threshold after fmodel & with lowpass neuron after error)
 	      //3 == delay embedded systems 2D

 	     //Switch on delay embedded systems 2D
 	      switchon_ED = false;

 	      //Switch on or off reflexes
 	      switchon_reflexes = false;// true==on, false == off

 	      //Switch on pure foot signal
 	      switchon_purefootsignal = false;//false;//true; // 1==on using only foot signal, 0 == using forward model & foot signal

 	      //Used learn weights
 	      switchon_learnweights = true;// 1== used learn weight, 0 == initial weights = 0.0 and let learning develops weights

 	      //Switch on foot inhibition
 	      switchon_footinhibition = false; //true = hind foot inhibit front foot, false;

 	      //Switch on soft landing  = reset to normal walking as  soon as acc error = 0
 	      softlanding = false;//true;

 	      //Testing controller from text (e.g. SOINN control as motor memory network)
 	     reading_text_testing = false;


 	  };


NeuralLocomotionControlAdaptiveClimbing::~NeuralLocomotionControlAdaptiveClimbing(){


		  //Save files
		  //outFilenlc1.close();
		  //outFilenlcr.close();
	 	  //outFilenlft.close();

  	};

std::vector<double> NeuralLocomotionControlAdaptiveClimbing::step_nlc(const std::vector<double> in0, const std::vector<double> in1){//, bool Footinhibition){

	   // Define local parameters//
		//std::vector<double> m;

		//m.resize(19); //  number of motors


	   //Input to control locomotion
		input.at(0) = 0; //walking = 0, joint inhibition = 1
		input.at(1) = 1; //lateral = 0, no lateral motion = 1
		input.at(2) = 1; //lateral right = 0, lateral left = 1
		input.at(3) = -1; //turn left = 1,
		input.at(4) = -1; //turn right = 1,

		// --------------deviation signal by Ren--------------------
		alpha_old =  alpha;

		if (sign(in0[G0x_s])>0)
		{ // goal is in front
			alpha = atan(in0[G0y_s]/in0[G0x_s]) * 180 / M_PI; // angle in degrees
		}
		else
		{ // behind robot angles "continue", directly behind robot is 180 degrees, left is negative, right is positive
			alpha_tmp = -1*atan (in0[G0y_s]/in0[G0x_s]) * 180 / M_PI; // angle in degrees
			if (alpha_tmp<=0)
			{ // left
				alpha = (-90 + (-90-alpha_tmp));
			}
			else
			{ // right
				alpha = ( 90 + ( 90-alpha_tmp));
			}
		}
		delta_alpha = alpha - alpha_old;
		Deviation_of_y = in0[G0y_s];
		// --------------deviation signal by Ren--------------------

/*******************************************************************************
 *  MODULE 1 CPG
 *******************************************************************************/

		//**********CPG***************
		//From 0.02-1.5
		//Control_input = 0.02;// slow Wave
		//Control_input = 0.03;// slow Wave OK USED
		//Control_input = 0.05;//slow stable Tetrapod OK USED
		//Control_input = 0.14; //terapod OK USED
		//Control_input = 0.18; //Tripod fast OK USED
		//Control_input = 0.34; //Faster than tripod

/*		cpg_w.at(0).at(0) =  1.4;
		cpg_w.at(0).at(1) =  0.18+Control_input;//0.4;
	    cpg_w.at(1).at(0) =  -0.18-Control_input;//-0.4
		cpg_w.at(1).at(1) =  1.4;

		cpg_activity.at(0) = cpg_w.at(0).at(0) * cpg_output.at(0) + cpg_w.at(0).at(1) * cpg_output.at(1) + cpg_bias;
		cpg_activity.at(1) = cpg_w.at(1).at(1) * cpg_output.at(1) + cpg_w.at(1).at(0) * cpg_output.at(0) + cpg_bias;

		for(unsigned int i=0; i<cpg_output.size();i++)
		{
		cpg_output.at(i) = tanh(cpg_activity.at(i));
		}*/
		//********CPG end************
		

	//--------------------------------------------------------------------------------
	//--------------------------ChaosCPG + post processing----------------------------
	//--------------------------------------------------------------------------------
	//starting!
	LegDamage_flag[LEG_RF] = true;
	if(global_count == 0)
	{
		ChaosCont.memory = 0;//10000;
 		ChaosCont.memory1 = 0;//10000;
 		ChaosCont.memory2 = 0;//10000;
 		ChaosCont.memory3 = 0;//10000;
 		ChaosCont.memory4 = 0;//10000;
 		ChaosCont.synflag = false;//true;
 		ChaosCont.synflag1 = false;//true;
 		ChaosCont.synflag2 = false;//true;
 		ChaosCont.synflag3 = false;//true;
 		ChaosCont.synflag4 = false;//true;
 		ChaosCont.setPeriod(8);			//RF
 		ChaosCont.setPeriod_cli(8);		//RM
 		ChaosCont.setPeriod_cli1(8);	//RH
 		ChaosCont.setPeriod_cli2(9);	//LF
 		ChaosCont.setPeriod_cli3(9);	//LM
 		ChaosCont.setPeriod_cli4(9);	//LH
 		
	}
	if( in0[G0x_s] < 3 )
	{
		cout<<"The deviation in y direction is "<<in0[G0y_s]<<endl;
		cout<<"The deviation angle is "<<alpha<<endl;
	}
	/*if(global_count == 600)
	{
 		ChaosCont.memory = 0;
 		ChaosCont.memory1 = 0;
 		ChaosCont.synflag = false;
 		ChaosCont.synflag1 = false;
 		ChaosCont.setPeriod(4);
 		ChaosCont.setPeriod_cli(8);
 		ChaosCont.setPeriod_cli1(9);
	}
 	if(global_count == 1200)
 	{
 		ChaosCont.memory = 10000;
 		ChaosCont.memory1 = 10000;
 		ChaosCont.synflag = true;
 		ChaosCont.synflag1 = true;
 		ChaosCont.setPeriod(8);
 		ChaosCont.setPeriod_cli(5);
 		ChaosCont.setPeriod_cli1(9);
 	}*/
 
	//ChaosControl CPG
	ChaosCont.ChaosControl();
	cpg_output.at(0) = ChaosCont.getOutput1();
	cpg_output.at(1) = ChaosCont.getOutput2();
	cpg_output1.at(0) = ChaosCont.getOutput1_cli();
	cpg_output1.at(1) = ChaosCont.getOutput2_cli();
	cpg_output2.at(0) = ChaosCont.getOutput1_cli1();
	cpg_output2.at(1) = ChaosCont.getOutput2_cli1();
	cpg_output3.at(0) = ChaosCont.getOutput1_cli2();
	cpg_output3.at(1) = ChaosCont.getOutput2_cli2();
	cpg_output4.at(0) = ChaosCont.getOutput1_cli3();
	cpg_output4.at(1) = ChaosCont.getOutput2_cli3();
	cpg_output5.at(0) = ChaosCont.getOutput1_cli4();
	cpg_output5.at(1) = ChaosCont.getOutput2_cli4();


/*******************************************************************************
*  MODULE 3 PSN
*******************************************************************************/

		//**********PSN***************
		psn_activity.at(0) = psn_input2_w.at(0).at(0) * input.at(2) + psn_bias.at(0);
		psn_activity.at(1) = psn_input2_w.at(1).at(0) * input.at(2);

		psn_activity.at(2) = psn_pcpg_w.at(2).at(0) * cpg_output.at(1) + psn_w.at(2).at(0) * psn_output.at(0);
		psn_activity.at(3) = psn_pcpg_w.at(3).at(1) * cpg_output.at(0) + psn_w.at(3).at(1) * psn_output.at(1);
		psn_activity.at(4) = psn_pcpg_w.at(4).at(1) * cpg_output.at(0) + psn_w.at(4).at(0) * psn_output.at(0);
		psn_activity.at(5) = psn_pcpg_w.at(5).at(0) * cpg_output.at(1) + psn_w.at(5).at(1) * psn_output.at(1);

		psn_activity.at(6) = psn_w.at(6).at(2) * psn_output.at(2) + psn_bias.at(1);
		psn_activity.at(7) = psn_w.at(7).at(3) * psn_output.at(3) + psn_bias.at(1);
		psn_activity.at(8) = psn_w.at(8).at(4) * psn_output.at(4) + psn_bias.at(1);
		psn_activity.at(9) = psn_w.at(9).at(5) * psn_output.at(5) + psn_bias.at(1);

		psn_activity.at(10) = psn_w.at(10).at(6) * psn_output.at(6) + psn_w.at(10).at(7) * psn_output.at(7) + psn_bias.at(2); // final output to motors
		psn_activity.at(11) = psn_w.at(11).at(8) * psn_output.at(8) + psn_w.at(11).at(9) * psn_output.at(9) + psn_bias.at(2); // final output to VRN

		for(unsigned int i=0; i<psn_output.size();i++)
		{
		psn_output.at(i) = tanh(psn_activity.at(i));
		}
	   	//********PSN end************

		//**********PSN1*************** --guanjiao
		psn_activity1.at(0) = psn_input2_w1.at(0).at(0) * input.at(2) + psn_bias1.at(0);
		psn_activity1.at(1) = psn_input2_w1.at(1).at(0) * input.at(2);

		psn_activity1.at(2) = psn_pcpg_w1.at(2).at(0) * cpg_output1.at(1) + psn_w1.at(2).at(0) * psn_output1.at(0);
		psn_activity1.at(3) = psn_pcpg_w1.at(3).at(1) * cpg_output1.at(0) + psn_w1.at(3).at(1) * psn_output1.at(1);
		psn_activity1.at(4) = psn_pcpg_w1.at(4).at(1) * cpg_output1.at(0) + psn_w1.at(4).at(0) * psn_output1.at(0);
		psn_activity1.at(5) = psn_pcpg_w1.at(5).at(0) * cpg_output1.at(1) + psn_w1.at(5).at(1) * psn_output1.at(1);

		psn_activity1.at(6) = psn_w1.at(6).at(2) * psn_output1.at(2) + psn_bias1.at(1);
		psn_activity1.at(7) = psn_w1.at(7).at(3) * psn_output1.at(3) + psn_bias1.at(1);
		psn_activity1.at(8) = psn_w1.at(8).at(4) * psn_output1.at(4) + psn_bias1.at(1);
		psn_activity1.at(9) = psn_w1.at(9).at(5) * psn_output1.at(5) + psn_bias1.at(1);

		psn_activity1.at(10) = psn_w1.at(10).at(6) * psn_output1.at(6) + psn_w1.at(10).at(7) * psn_output1.at(7) + psn_bias1.at(2); // final output to motors
		psn_activity1.at(11) = psn_w1.at(11).at(8) * psn_output1.at(8) + psn_w1.at(11).at(9) * psn_output1.at(9) + psn_bias1.at(2); // final output to VRN

		for(unsigned int i=0; i<psn_output1.size();i++)
		{
		psn_output1.at(i) = tanh(psn_activity1.at(i));
		}
	   	//********PSN1 end************
	   
	   	//**********PSN2***************
		psn_activity2.at(0) = psn_input2_w2.at(0).at(0) * input.at(2) + psn_bias2.at(0);
		psn_activity2.at(1) = psn_input2_w2.at(1).at(0) * input.at(2);

		psn_activity2.at(2) = psn_pcpg_w2.at(2).at(0) * cpg_output2.at(1) + psn_w2.at(2).at(0) * psn_output2.at(0);
		psn_activity2.at(3) = psn_pcpg_w2.at(3).at(1) * cpg_output2.at(0) + psn_w2.at(3).at(1) * psn_output2.at(1);
		psn_activity2.at(4) = psn_pcpg_w2.at(4).at(1) * cpg_output2.at(0) + psn_w2.at(4).at(0) * psn_output2.at(0);
		psn_activity2.at(5) = psn_pcpg_w2.at(5).at(0) * cpg_output2.at(1) + psn_w2.at(5).at(1) * psn_output2.at(1);

		psn_activity2.at(6) = psn_w2.at(6).at(2) * psn_output2.at(2) + psn_bias2.at(1);
		psn_activity2.at(7) = psn_w2.at(7).at(3) * psn_output2.at(3) + psn_bias2.at(1);
		psn_activity2.at(8) = psn_w2.at(8).at(4) * psn_output2.at(4) + psn_bias2.at(1);
		psn_activity2.at(9) = psn_w2.at(9).at(5) * psn_output2.at(5) + psn_bias2.at(1);

		psn_activity2.at(10) = psn_w2.at(10).at(6) * psn_output2.at(6) + psn_w2.at(10).at(7) * psn_output2.at(7) + psn_bias2.at(2); // final output to motors
		psn_activity2.at(11) = psn_w2.at(11).at(8) * psn_output2.at(8) + psn_w2.at(11).at(9) * psn_output2.at(9) + psn_bias2.at(2); // final output to VRN

		for(unsigned int i=0; i<psn_output2.size();i++)
		{
		psn_output2.at(i) = tanh(psn_activity2.at(i));
		}
	   	//********PSN2 end************
	   	
	   	//**********PSN3*************** --guanjiao
		psn_activity3.at(0) = psn_input2_w3.at(0).at(0) * input.at(2) + psn_bias3.at(0);
		psn_activity3.at(1) = psn_input2_w3.at(1).at(0) * input.at(2);

		psn_activity3.at(2) = psn_pcpg_w3.at(2).at(0) * cpg_output3.at(1) + psn_w3.at(2).at(0) * psn_output3.at(0);
		psn_activity3.at(3) = psn_pcpg_w3.at(3).at(1) * cpg_output3.at(0) + psn_w3.at(3).at(1) * psn_output3.at(1);
		psn_activity3.at(4) = psn_pcpg_w3.at(4).at(1) * cpg_output3.at(0) + psn_w3.at(4).at(0) * psn_output3.at(0);
		psn_activity3.at(5) = psn_pcpg_w3.at(5).at(0) * cpg_output3.at(1) + psn_w3.at(5).at(1) * psn_output3.at(1);

		psn_activity3.at(6) = psn_w3.at(6).at(2) * psn_output3.at(2) + psn_bias3.at(1);
		psn_activity3.at(7) = psn_w3.at(7).at(3) * psn_output3.at(3) + psn_bias3.at(1);
		psn_activity3.at(8) = psn_w3.at(8).at(4) * psn_output3.at(4) + psn_bias3.at(1);
		psn_activity3.at(9) = psn_w3.at(9).at(5) * psn_output3.at(5) + psn_bias3.at(1);

		psn_activity3.at(10) = psn_w3.at(10).at(6) * psn_output3.at(6) + psn_w3.at(10).at(7) * psn_output3.at(7) + psn_bias3.at(2); // final output to motors
		psn_activity3.at(11) = psn_w3.at(11).at(8) * psn_output3.at(8) + psn_w3.at(11).at(9) * psn_output3.at(9) + psn_bias3.at(2); // final output to VRN

		for(unsigned int i=0; i<psn_output3.size();i++)
		{
		psn_output3.at(i) = tanh(psn_activity3.at(i));
		}
	   	//********PSN3 end************
	   
	   	//**********PSN4***************
		psn_activity4.at(0) = psn_input2_w4.at(0).at(0) * input.at(2) + psn_bias4.at(0);
		psn_activity4.at(1) = psn_input2_w4.at(1).at(0) * input.at(2);

		psn_activity4.at(2) = psn_pcpg_w4.at(2).at(0) * cpg_output4.at(1) + psn_w4.at(2).at(0) * psn_output4.at(0);
		psn_activity4.at(3) = psn_pcpg_w4.at(3).at(1) * cpg_output4.at(0) + psn_w4.at(3).at(1) * psn_output4.at(1);
		psn_activity4.at(4) = psn_pcpg_w4.at(4).at(1) * cpg_output4.at(0) + psn_w4.at(4).at(0) * psn_output4.at(0);
		psn_activity4.at(5) = psn_pcpg_w4.at(5).at(0) * cpg_output4.at(1) + psn_w4.at(5).at(1) * psn_output4.at(1);

		psn_activity4.at(6) = psn_w4.at(6).at(2) * psn_output4.at(2) + psn_bias4.at(1);
		psn_activity4.at(7) = psn_w4.at(7).at(3) * psn_output4.at(3) + psn_bias4.at(1);
		psn_activity4.at(8) = psn_w4.at(8).at(4) * psn_output4.at(4) + psn_bias4.at(1);
		psn_activity4.at(9) = psn_w4.at(9).at(5) * psn_output4.at(5) + psn_bias4.at(1);

		psn_activity4.at(10) = psn_w4.at(10).at(6) * psn_output4.at(6) + psn_w4.at(10).at(7) * psn_output4.at(7) + psn_bias4.at(2); // final output to motors
		psn_activity4.at(11) = psn_w4.at(11).at(8) * psn_output4.at(8) + psn_w4.at(11).at(9) * psn_output4.at(9) + psn_bias4.at(2); // final output to VRN

		for(unsigned int i=0; i<psn_output4.size();i++)
		{
		psn_output4.at(i) = tanh(psn_activity4.at(i));
		}
	   	//********PSN4 end************
	   	
	   	//**********PSN5***************
		psn_activity5.at(0) = psn_input2_w5.at(0).at(0) * input.at(2) + psn_bias5.at(0);
		psn_activity5.at(1) = psn_input2_w5.at(1).at(0) * input.at(2);

		psn_activity5.at(2) = psn_pcpg_w5.at(2).at(0) * cpg_output5.at(1) + psn_w5.at(2).at(0) * psn_output5.at(0);
		psn_activity5.at(3) = psn_pcpg_w5.at(3).at(1) * cpg_output5.at(0) + psn_w5.at(3).at(1) * psn_output5.at(1);
		psn_activity5.at(4) = psn_pcpg_w5.at(4).at(1) * cpg_output5.at(0) + psn_w5.at(4).at(0) * psn_output5.at(0);
		psn_activity5.at(5) = psn_pcpg_w5.at(5).at(0) * cpg_output5.at(1) + psn_w5.at(5).at(1) * psn_output5.at(1);

		psn_activity5.at(6) = psn_w5.at(6).at(2) * psn_output5.at(2) + psn_bias5.at(1);
		psn_activity5.at(7) = psn_w5.at(7).at(3) * psn_output5.at(3) + psn_bias5.at(1);
		psn_activity5.at(8) = psn_w5.at(8).at(4) * psn_output5.at(4) + psn_bias5.at(1);
		psn_activity5.at(9) = psn_w5.at(9).at(5) * psn_output5.at(5) + psn_bias5.at(1);

		psn_activity5.at(10) = psn_w5.at(10).at(6) * psn_output5.at(6) + psn_w5.at(10).at(7) * psn_output5.at(7) + psn_bias5.at(2); // final output to motors
		psn_activity5.at(11) = psn_w5.at(11).at(8) * psn_output5.at(8) + psn_w5.at(11).at(9) * psn_output5.at(9) + psn_bias5.at(2); // final output to VRN

		for(unsigned int i=0; i<psn_output5.size();i++)
		{
		psn_output5.at(i) = tanh(psn_activity5.at(i));
		}
	   	//********PSN5 end************


/*******************************************************************************
*  MODULE 4 VRNs
*******************************************************************************/

	   //**********VRN***************

		//VRN 1 (left)
		vrn_activity.at(0) = vrn_psn_w.at(0).at(11) * psn_output3.at(11);
		vrn_activity.at(1) = vrn_input3_w * input.at(3);

		vrn_activity.at(4) = vrn_w.at(4).at(0) * vrn_output.at(0) + vrn_w.at(4).at(1) * vrn_output.at(1) + vrn_bias;
		vrn_activity.at(5) = vrn_w.at(5).at(0) * vrn_output.at(0) + vrn_w.at(5).at(1) * vrn_output.at(1) + vrn_bias;
		vrn_activity.at(6) = vrn_w.at(6).at(0) * vrn_output.at(0) + vrn_w.at(6).at(1) * vrn_output.at(1) + vrn_bias;
		vrn_activity.at(7) = vrn_w.at(7).at(0) * vrn_output.at(0) + vrn_w.at(7).at(1) * vrn_output.at(1) + vrn_bias;

		vrn_activity.at(12) = vrn_w.at(12).at(4) * vrn_output.at(4) + vrn_w.at(12).at(5) * vrn_output.at(5) + vrn_w.at(12).at(6) * vrn_output.at(6)
				+ vrn_w.at(12).at(7) * vrn_output.at(7); //Output to TL1,2,3

		//VRN 2 (right)
		vrn_activity.at(2) = vrn_psn_w.at(2).at(11) * psn_output.at(11);
		vrn_activity.at(3) = vrn_input4_w * input.at(4);

		vrn_activity.at(8) = vrn_w.at(8).at(2) * vrn_output.at(2) + vrn_w.at(8).at(3) * vrn_output.at(3) + vrn_bias;
		vrn_activity.at(9) = vrn_w.at(9).at(2) * vrn_output.at(2) + vrn_w.at(9).at(3) * vrn_output.at(3) + vrn_bias;
		vrn_activity.at(10) = vrn_w.at(10).at(2) * vrn_output.at(2) + vrn_w.at(10).at(3) * vrn_output.at(3) + vrn_bias;
		vrn_activity.at(11) = vrn_w.at(11).at(2) * vrn_output.at(2) + vrn_w.at(11).at(3) * vrn_output.at(3) + vrn_bias;

		vrn_activity.at(13) = vrn_w.at(13).at(8) * vrn_output.at(8) + vrn_w.at(13).at(9) * vrn_output.at(9) + vrn_w.at(13).at(10) * vrn_output.at(10)
				+ vrn_w.at(13).at(11) * vrn_output.at(11); //Output to TR1,2,3


		for(unsigned int i=0; i<vrn_output.size();i++)
		{
		vrn_output.at(i) = tanh(vrn_activity.at(i));
		}

	    //*******VRN end***************

	   //**********VRN1*************** --guanjiao

		//VRN 1 (left)
		vrn_activity1.at(0) = vrn_psn_w1.at(0).at(11) * psn_output4.at(11);
		vrn_activity1.at(1) = vrn_input3_w1 * input.at(3);

		vrn_activity1.at(4) = vrn_w1.at(4).at(0) * vrn_output1.at(0) + vrn_w1.at(4).at(1) * vrn_output1.at(1) + vrn_bias1;
		vrn_activity1.at(5) = vrn_w1.at(5).at(0) * vrn_output1.at(0) + vrn_w1.at(5).at(1) * vrn_output1.at(1) + vrn_bias1;
		vrn_activity1.at(6) = vrn_w1.at(6).at(0) * vrn_output1.at(0) + vrn_w1.at(6).at(1) * vrn_output1.at(1) + vrn_bias1;
		vrn_activity1.at(7) = vrn_w1.at(7).at(0) * vrn_output1.at(0) + vrn_w1.at(7).at(1) * vrn_output1.at(1) + vrn_bias1;

		vrn_activity1.at(12) = vrn_w1.at(12).at(4) * vrn_output1.at(4) + vrn_w1.at(12).at(5) * vrn_output1.at(5) + vrn_w1.at(12).at(6) * vrn_output1.at(6)
				+ vrn_w1.at(12).at(7) * vrn_output1.at(7); //Output to TL1,2,3

		//VRN 2 (right)
		vrn_activity1.at(2) = vrn_psn_w1.at(2).at(11) * psn_output1.at(11);
		vrn_activity1.at(3) = vrn_input4_w1 * input.at(4);

		vrn_activity1.at(8) = vrn_w1.at(8).at(2) * vrn_output1.at(2) + vrn_w1.at(8).at(3) * vrn_output1.at(3) + vrn_bias1;
		vrn_activity1.at(9) = vrn_w1.at(9).at(2) * vrn_output1.at(2) + vrn_w1.at(9).at(3) * vrn_output1.at(3) + vrn_bias1;
		vrn_activity1.at(10) = vrn_w1.at(10).at(2) * vrn_output1.at(2) + vrn_w1.at(10).at(3) * vrn_output1.at(3) + vrn_bias1;
		vrn_activity1.at(11) = vrn_w1.at(11).at(2) * vrn_output1.at(2) + vrn_w1.at(11).at(3) * vrn_output1.at(3) + vrn_bias1;

		vrn_activity1.at(13) = vrn_w1.at(13).at(8) * vrn_output1.at(8) + vrn_w1.at(13).at(9) * vrn_output1.at(9) + vrn_w1.at(13).at(10) * vrn_output1.at(10)
				+ vrn_w1.at(13).at(11) * vrn_output1.at(11); //Output to TR1,2,3


		for(unsigned int i=0; i<vrn_output1.size();i++)
		{
		vrn_output1.at(i) = tanh(vrn_activity1.at(i));
		}

	    //*******VRN1 end***************
	    
	   //**********VRN2***************

		//VRN 1 (left)
		vrn_activity2.at(0) = vrn_psn_w2.at(0).at(11) * psn_output5.at(11);
		vrn_activity2.at(1) = vrn_input3_w2 * input.at(3);

		vrn_activity2.at(4) = vrn_w2.at(4).at(0) * vrn_output2.at(0) + vrn_w2.at(4).at(1) * vrn_output2.at(1) + vrn_bias2;
		vrn_activity2.at(5) = vrn_w2.at(5).at(0) * vrn_output2.at(0) + vrn_w2.at(5).at(1) * vrn_output2.at(1) + vrn_bias2;
		vrn_activity2.at(6) = vrn_w2.at(6).at(0) * vrn_output2.at(0) + vrn_w2.at(6).at(1) * vrn_output2.at(1) + vrn_bias2;
		vrn_activity2.at(7) = vrn_w2.at(7).at(0) * vrn_output2.at(0) + vrn_w2.at(7).at(1) * vrn_output2.at(1) + vrn_bias2;

		vrn_activity2.at(12) = vrn_w2.at(12).at(4) * vrn_output2.at(4) + vrn_w2.at(12).at(5) * vrn_output2.at(5) + vrn_w2.at(12).at(6) * vrn_output2.at(6)
				+ vrn_w2.at(12).at(7) * vrn_output2.at(7); //Output to TL1,2,3

		//VRN 2 (right)
		vrn_activity2.at(2) = vrn_psn_w2.at(2).at(11) * psn_output2.at(11);
		vrn_activity2.at(3) = vrn_input4_w2 * input.at(4);

		vrn_activity2.at(8) = vrn_w2.at(8).at(2) * vrn_output2.at(2) + vrn_w2.at(8).at(3) * vrn_output2.at(3) + vrn_bias2;
		vrn_activity2.at(9) = vrn_w2.at(9).at(2) * vrn_output2.at(2) + vrn_w2.at(9).at(3) * vrn_output2.at(3) + vrn_bias2;
		vrn_activity2.at(10) = vrn_w2.at(10).at(2) * vrn_output2.at(2) + vrn_w2.at(10).at(3) * vrn_output2.at(3) + vrn_bias2;
		vrn_activity2.at(11) = vrn_w2.at(11).at(2) * vrn_output2.at(2) + vrn_w2.at(11).at(3) * vrn_output2.at(3) + vrn_bias2;

		vrn_activity2.at(13) = vrn_w2.at(13).at(8) * vrn_output2.at(8) + vrn_w2.at(13).at(9) * vrn_output2.at(9) + vrn_w2.at(13).at(10) * vrn_output2.at(10)
				+ vrn_w2.at(13).at(11) * vrn_output2.at(11); //Output to TR1,2,3


		for(unsigned int i=0; i<vrn_output2.size();i++)
		{
		vrn_output2.at(i) = tanh(vrn_activity2.at(i));
		}

	    //*******VRN2 end***************

/*******************************************************************************
*  MODULE 5 PRE-MOTOR NEURONS
*******************************************************************************/

		//*******Motor neurons**********

		//-----Diff CL, CR-------------

		for(unsigned int i=0; i<cr_output.size();i++)
		{
		cr_outputold.at(i) = cr_output.at(i);
		}
		for(unsigned int i=0; i<cl_output.size();i++)
		{
		cl_outputold.at(i) = cl_output.at(i);
		}
		//-----Diff CL, CR end--------


		//with CPG

		//Tripod wiring
/*		tr_activity.at(0) = -2.5*vrn_output.at(13)+10*input.at(0);
		tr_activity.at(1) = 2.5*vrn_output.at(13)+(-10)*input.at(0);
		tr_activity.at(2) = -2.5*vrn_output.at(13)+(-10)*input.at(0);

		tl_activity.at(0) = 2.5*vrn_output.at(12)+10*input.at(0);
		tl_activity.at(1) = -2.5*vrn_output.at(12)+(-10)*input.at(0);
		tl_activity.at(2) = 2.5*vrn_output.at(12)+(-10)*input.at(0);
*/

		/*/-----------------------------One neuron Control ---------------------------//
		tr_activity.at(0) = -2.5*vrn_output.at(13)+10*input.at(0);
		tr_activity.at(1) = -2.5*vrn_output.at(13)+(-10)*input.at(0);
		tr_activity.at(2) = -2.5*vrn_output.at(13)+(-10)*input.at(0);/////////////USED

		tl_activity.at(0) = -2.5*vrn_output.at(12)+10*input.at(0);
		tl_activity.at(1) = -2.5*vrn_output.at(12)+(-10)*input.at(0);
		tl_activity.at(2) = -2.5*vrn_output.at(12)+(-10)*input.at(0);


		cl_activity.at(0) = -5.0*psn_output.at(11)-1.0+10*input.at(0);
		cl_activity.at(1) =  5.0*psn_output.at(11)-1.0+10*input.at(0);
		cl_activity.at(2) = -5.0*psn_output.at(11)-1.0+10*input.at(0);

		cr_activity.at(0) =  5.0*psn_output.at(11)-1.0+10*input.at(0);
		cr_activity.at(1) = -5.0*psn_output.at(11)-1.0+10*input.at(0);
		cr_activity.at(2) =  1.0*psn_output.at(11)+10.0+10*input.at(0);//////////////USED

		fl_activity.at(0) = -2.2*psn_output.at(10)*input.at(1)+1+10*input.at(0);//+(-2)*input.at(1);
		fl_activity.at(1) = 2.2*psn_output.at(10)*input.at(1)+1+10*input.at(0);//+(-2)*input.at(1);
		fl_activity.at(2) = -2.2*psn_output.at(10)*input.at(1)+1+10*input.at(0);//+(-2)*input.at(1);

		fr_activity.at(0) = -2.2*psn_output.at(10)*input.at(1)+1+10*input.at(0);//+(-2)*input.at(1);
		fr_activity.at(1) = 2.2*psn_output.at(10)*input.at(1)+1+10*input.at(0);//+(-2)*input.at(1);
		fr_activity.at(2) = 1.0*psn_output.at(11)-0.5+10*input.at(0);////////////USED
		//-----------------------------One neuron Control end---------------------------/*/
		
/*		fr_activity.at(0) = -2.2*psn_output.at(10)+1+10*input.at(0)+(-2)*input.at(1);
		fr_activity.at(1) = 2.2*psn_output.at(10)+1+10*input.at(0)+(-2)*input.at(1);
		fr_activity.at(2) = -2.2*psn_output.at(10)+1+10*input.at(0)+(-2)*input.at(1);

		fl_activity.at(0) = -2.2*psn_output.at(10)+1+10*input.at(0)+(-2)*input.at(1);
		fl_activity.at(1) = 2.2*psn_output.at(10)+1+10*input.at(0)+(-2)*input.at(1);
		fl_activity.at(2) = -2.2*psn_output.at(10)+1+10*input.at(0)+(-2)*input.at(1);
*/

		//-----------------------------Three neuron Control ---------------------------//
		tr_activity.at(0) = -2.5*vrn_output.at(13)+10*input.at(0);
		tr_activity.at(1) = -2.5*vrn_output1.at(13)+(-10)*input.at(0);
		tr_activity.at(2) = -2.5*vrn_output2.at(13)+(-10)*input.at(0);/////////////USED

		tl_activity.at(0) = -2.5*vrn_output.at(12)+10*input.at(0);
		tl_activity.at(1) = -2.5*vrn_output1.at(12)+(-10)*input.at(0);
		tl_activity.at(2) = -2.5*vrn_output2.at(12)+(-10)*input.at(0);

		//for the CTr- joint, weight should be +, bias should be -
		//weight = 5, bias = -0.5
		cl_activity.at(0) = 5*psn_output3.at(11)-0.5+10*input.at(0);
		cl_activity.at(1) = 5*psn_output4.at(11)-0.5+10*input.at(0);
		cl_activity.at(2) = 5*psn_output5.at(11)-0.5+10*input.at(0);

		cr_activity.at(0) = 5*psn_output.at(11)-0.5+10*input.at(0);
		cr_activity.at(1) = 5*psn_output1.at(11)-0.5+10*input.at(0);
		cr_activity.at(2) = 5*psn_output2.at(11)-0.5+10*input.at(0);

		fl_activity.at(0) = 5*psn_output3.at(11)*input.at(1)-0.5+10*input.at(0);//+(-2)*input.at(1);
		fl_activity.at(1) = 5*psn_output4.at(11)*input.at(1)-0.5+10*input.at(0);//+(-2)*input.at(1);
		fl_activity.at(2) = 5*psn_output5.at(11)*input.at(1)-0.5+10*input.at(0);//+(-2)*input.at(1);

		fr_activity.at(0) = 5*psn_output.at(11)*input.at(1)-0.5+10*input.at(0);//+(-2)*input.at(1);
		fr_activity.at(1) = 5*psn_output1.at(11)*input.at(1)-0.5+10*input.at(0);//+(-2)*input.at(1);
		fr_activity.at(2) = 5*psn_output2.at(11)*input.at(1)-0.5+10*input.at(0);//+(-2)*input.at(1);
		//-----------------------------Three neuron Control end---------------------------//


		//-----CONTROL BACKBONE JOINT HERE------------------------------------------//

		bj_activity.at(0) = 0.0;//-0.05;//tr_activity.at(2);

		//-----CONTROL BACKBONE JOINT HERE------------------------------------------//

		for(unsigned int i=0; i<tr_output.size();i++)
		{
		tr_output.at(i) = tanh(tr_activity.at(i));
		}

		for(unsigned int i=0; i<tl_output.size();i++)
		{
		tl_output.at(i) = tanh(tl_activity.at(i));
		}

		for(unsigned int i=0; i<cr_output.size();i++)
		{
		cr_output.at(i) = tanh(cr_activity.at(i));
		}

		for(unsigned int i=0; i<cl_output.size();i++)
		{
		cl_output.at(i) = tanh(cl_activity.at(i));
		}

		for(unsigned int i=0; i<fr_output.size();i++)
		{
		fr_output.at(i) = tanh(fr_activity.at(i));
		}

		for(unsigned int i=0; i<fl_output.size();i++)
		{
		fl_output.at(i) = tanh(fl_activity.at(i));
		}

		bj_output.at(0) = tanh(bj_activity.at(0));

		//******Motor neurons end**********


		//*Scaling, shaping, and delay line****

		/*push_back = add one more element in the last with
		a value of the given variable, e.g., "buffer_t" will
		have one more element with the value of "tr_output.at(2)"
		*/

		buffer_t.push_back (tr_output.at(0));
		buffer_c.push_back (cr_output.at(0));
		buffer_f.push_back (fr_output.at(0));
		buffer_tl.push_back(tl_output.at(0));
		buffer_cl.push_back(cl_output.at(0));
		buffer_fl.push_back(fl_output.at(0));
		
		//added by guanjiao
		buffer_t1.push_back (tr_output.at(1));
		buffer_c1.push_back (cr_output.at(1));
		buffer_f1.push_back (fr_output.at(1));
		buffer_tl1.push_back(tl_output.at(1));
		buffer_cl1.push_back(cl_output.at(1));
		buffer_fl1.push_back(fl_output.at(1));
		
		buffer_t2.push_back (tr_output.at(2));
		buffer_c2.push_back (cr_output.at(2));
		buffer_f2.push_back (fr_output.at(2));
		buffer_tl2.push_back(tl_output.at(2));
		buffer_cl2.push_back(cl_output.at(2));
		buffer_fl2.push_back(fl_output.at(2));

//		buffer_tr.push_back (tr_output.at(2));
//		buffer_tl.push_back (tl_output.at(2));
//		buffer_c.push_back (cr_output.at(2));
//		buffer_f.push_back (fr_output.at(2));


		switch(option_wiring)
		{
		case 1:

			//-----Diff CL, CR-------------

			for(unsigned int i=0; i<cr_output.size();i++)
			{
				diffcr_output.at(i) = cr_output.at(i)-cr_outputold.at(i);
			}
			for(unsigned int i=0; i<cl_output.size();i++)
			{
				diffcl_output.at(i) = cl_output.at(i)-cl_outputold.at(i);
			}

			//postcl.at(0) = cl_output.at(0);

			for(unsigned int i=0; i<postcl.size();i++)
			{
				if(diffcl_output.at(i)<-0.02)// || cl_output.at(i)<0.0)
				{
					postcl.at(i) = -1;
				}
				if(diffcl_output.at(i)>=-0.02)
				{
					postcl.at(i) = cl_output.at(i);
				}
			}

			for(unsigned int i=0; i<postcr.size();i++)
			{
				if(diffcr_output.at(i)<-0.02)// || cl_output.at(i)<0.0)
				{
					postcr.at(i) = -1;
				}
				if(diffcr_output.at(i)>=-0.02)
				{
					postcr.at(i) = cr_output.at(i);
				}
			}


			//-----Diff CL, CR end--------



			//******Wiring wth ONLY Tripod******

			m_pre.at(TR0_m) = tr_output.at(0);
			m_pre.at(TR1_m) = tr_output.at(1);
			m_pre.at(TR2_m) = tr_output.at(2);

			m_pre.at(TL0_m) = tl_output.at(0);
			m_pre.at(TL1_m) = tl_output.at(1);
			m_pre.at(TL2_m) = tl_output.at(2);

			m_pre.at(CL0_m) = postcl.at(0);//cl_output.at(0);
			m_pre.at(CL1_m) = postcl.at(1);//cl_output.at(1);
			m_pre.at(CL2_m) = postcl.at(2);//cl_output.at(2);

			m_pre.at(CR0_m) = postcr.at(0);//cr_output.at(0);
			m_pre.at(CR1_m) = postcr.at(1);//cr_output.at(1);
			m_pre.at(CR2_m) = postcr.at(2);//cr_output.at(2);

			m_pre.at(FR0_m) = fr_output.at(0);
			m_pre.at(FR1_m) = fr_output.at(1);
			m_pre.at(FR2_m) = fr_output.at(2);

			m_pre.at(FL0_m) = fl_output.at(0);
			m_pre.at(FL1_m) = fl_output.at(1);
			m_pre.at(FL2_m) = fl_output.at(2);

			m.at(BJ_m) = bj_output.at(0);

			break;
			//******Wiring with ONLY Tripod end*

		case 2:
			//******Wiring with Delay line*******
			for(unsigned int i=0; i<postclold.size();i++)
			{
				postcrold.at(i) = postcr.at(i);
				postclold.at(i) = postcl.at(i);

			}


			//TR-------------------------

			if( (buffer_t.size() - 2*tau) > 0)
			{
				m_pre.at(TR0_m) = buffer_t[buffer_t.size() - 2*tau - 1];
			}
			else
			{
				m_pre.at(TR0_m) = 0;
				cout << "TR0 = 0" << endl;
			}

			if((buffer_t1.size() - tau)>0)
			{
				m_pre.at(TR1_m) = buffer_t1[buffer_t1.size() - tau -1];
			}
			else
			{
				m_pre.at(TR1_m) = 0;
			}

			if((buffer_t2.size() )>0)
			{
				m_pre.at(TR2_m) = buffer_t2[buffer_t2.size() - 1];
			}
			else
			{
				m_pre.at(TR2_m) = 0;
			}

			//TL-------------------------

			if((buffer_tl.size() - 2*tau - tau_l)>0)
			{
				m_pre.at(TL0_m) = buffer_tl[buffer_tl.size() - 2*tau - tau_l-1];
			}
			else
			{
				m_pre.at(TL0_m) = 0;
			}
			if((buffer_tl1.size()-tau-tau_l)>0)
			{
				m_pre.at(TL1_m) = buffer_tl1[buffer_tl1.size()-tau-tau_l-1];
			}
			else
			{
				m_pre.at(TL1_m) = 0;
			}
			if((buffer_tl2.size()- tau_l)>0)
			{
				m_pre.at(TL2_m) = buffer_tl2[buffer_tl2.size() -tau_l-1];
			}
			else
			{
				m_pre.at(TL2_m) = 0;
			}

			//CR-------------------------

			if((buffer_c.size()-2*tau)>0)
			{
				/*m_pre.at(CR0_m)*/ postcr.at(0) = buffer_c[buffer_c.size() - 2*tau - 1];
			}
			else
			{
				/* m_pre.at(CR0_m)*/ postcr.at(0)= 0;
			}
			/*******CHANGE KOH*post processing *******/
			m_pre.at(CR0_m) = postcr.at(0);
			if(postcr.at(0)<postcrold.at(0))//postcr.at(0)<threshold_c)
				m_pre.at(CR0_m) = -1;
			
			/*if (switchon_footinhibition){
				if(in0.at(R1_fs)>-0.5)
					m_pre.at(CR0_m)	= -1;
				//std::cout<<"foot inhibition"<< "\n";
			}*/


			/*******CHANGE KOH*post processing *******/


			if((buffer_c1.size()-tau)>0)
			{
				/*m_pre.at(CR1_m)*/ postcr.at(1) = buffer_c1[buffer_c1.size()-tau-1];
			}
			else
			{
				/*m_pre.at(CR1_m)*/  postcr.at(1)= 0;
			}
			/*******CHANGE KOH*post processing *******/
			m_pre.at(CR1_m) = postcr.at(1);
			if(postcr.at(1)<postcrold.at(1))//postcr.at(1)<threshold_c)
				m_pre.at(CR1_m) = -1;

			/*if (switchon_footinhibition){
				if(in0.at(R2_fs)>-0.5)
					m_pre.at(CR1_m)	= -1;
			}*
			/*******CHANGE KOH*post processing *******/


			if((buffer_c2.size())>0)
			{
				/*m_pre.at(CR2_m)*/ postcr.at(2) = buffer_c2[buffer_c2.size()-1];
			}
			else
			{
				/*m_pre.at(CR2_m)*/ postcr.at(2) = 0;
			}
			/*******CHANGE KOH*post processing *******/
			m_pre.at(CR2_m) = postcr.at(2);
			if(postcr.at(2)<postcrold.at(2))//postcr.at(2)<threshold_c)
				m_pre.at(CR2_m) = -1;
			
			/*if (switchon_footinhibition){
				if(in0.at(L0_fs)>-0.5)
					m_pre.at(CR2_m)	= -1;
			}*/
			/*******CHANGE KOH*post processing *******/

			//CL-------------------------

			if((buffer_cl.size()-2*tau-tau_l)>0)
			{
				/*m_pre.at(CL0_m)*/ postcl.at(0) = buffer_cl[buffer_cl.size()-2*tau-tau_l-1];
			}
			else
			{
				/*m_pre.at(CL0_m)*/ postcl.at(0) = 0;
			}
			/*******CHANGE KOH*post processing *******/
			m_pre.at(CL0_m) = postcl.at(0);
			if(postcl.at(0)<postclold.at(0))//if(postcl.at(0)<threshold_c)
				m_pre.at(CL0_m) = -1;

			/*if (switchon_footinhibition){
				if(in0.at(L1_fs)>-0.5)
					m_pre.at(CL0_m)	= -1;
			}*/
			/*******CHANGE KOH*post processing *******/



			if((buffer_cl1.size()-tau-tau_l)>0)
			{
				/*m_pre.at(CL1_m)*/ postcl.at(1) = buffer_cl1[buffer_cl1.size()-tau-tau_l-1];
			}
			else
			{
				/*m_pre.at(CL1_m)*/ postcl.at(1) = 0;
			}
			/*******CHANGE KOH*post processing ******/
			m_pre.at(CL1_m) = postcl.at(1);
			if(postcl.at(1)<postclold.at(1))//if(postcl.at(1)<threshold_c)
				m_pre.at(CL1_m) = -1;

			/*if (switchon_footinhibition){
				if(in0.at(L2_fs)>-0.5)
					m_pre.at(CL1_m)	= -1;
			}*/
			/*******CHANGE KOH*post processing *******/


			if((buffer_cl2.size()-tau_l)>0)
			{
				/*m_pre.at(CL2_m)*/ postcl.at(2)  = buffer_cl2[buffer_cl2.size()-tau_l-1];
			}
			else
			{
				/*m_pre.at(CL2_m)*/ postcl.at(2) = 0;
			}
			/*******CHANGE KOH*post processing ******/
			m_pre.at(CL2_m) = postcl.at(2);
			if(postcl.at(2)<postclold.at(2))//if(postcl.at(2)<threshold_c)
				m_pre.at(CL2_m) = -1;

			/*if (switchon_footinhibition){
				if(in0.at(R0_fs)>-0.5)
					m_pre.at(CL2_m)	= -1;
			}*/
			/*******CHANGE KOH*post processing *******/


			//FR-------------------------


			if((buffer_f.size()-2*tau)>0)
			{
				m_pre.at(FR0_m) = buffer_f[buffer_f.size()-2*tau -1];
			}
			else
			{
				m_pre.at(FR0_m) = 0;
			}
			if((buffer_f1.size()-tau)>0)
			{
				m_pre.at(FR1_m) = buffer_f1[buffer_f1.size()-tau-1];
			}
			else
			{
				m_pre.at(FR1_m) = 0;
			}
			if((buffer_f2.size())>0)
			{
				m_pre.at(FR2_m) = -1*buffer_f2[buffer_f2.size()-1]; // Change! inverse signal
			}
			else
			{
				m_pre.at(FR2_m) = 0;
			}

			//FL-------------------------

			if((buffer_fl.size()-2*tau-tau_l)>0)
			{
				m_pre.at(FL0_m) = buffer_fl[buffer_fl.size()-2*tau-tau_l-1];
			}
			else
			{
				m_pre.at(FL0_m) = 0;
			}
			if((buffer_fl1.size()-tau-tau_l)>0)
			{
				m_pre.at(FL1_m) = buffer_fl1[buffer_fl1.size()-tau-tau_l-1];
			}
			else
			{
				m_pre.at(FL1_m) = 0;
			}
			if((buffer_fl2.size()-tau_l)>0)
			{
				m_pre.at(FL2_m) = -1*buffer_fl2[buffer_fl2.size()-tau_l-1];// Change! inverse signal
			}
			else
			{
				m_pre.at(FL2_m) = 0;
			}

			m_pre.at(BJ_m) = bj_output.at(0);

			/*if(buffer_t.size() % (10*tau) == 0)
			{
				buffer_t.erase (buffer_t.begin(), buffer_t.begin() + 5.0*tau);
				buffer_c.erase (buffer_c.begin(), buffer_c.begin() + 5.0*tau);
				buffer_f.erase (buffer_f.begin(), buffer_f.begin() + 5.0*tau);
			}*/

//		    	if(buffer_tr.size() % (4*tau) == 0)
//		    	{
//				buffer_tr.erase (buffer_tr.begin(), buffer_tr.begin() + 2.0*tau);
//
//		    	}
//
			if(buffer_c.size() % (6*tau) == 0)
			{
				//std::cout<<"testing"<<endl;
				buffer_t.erase (buffer_t.begin(), buffer_t.begin() + 3.0*tau);
				buffer_c.erase (buffer_c.begin(), buffer_c.begin() + 3.0*tau);
				buffer_f.erase (buffer_f.begin(), buffer_f.begin() + 3.0*tau);
				buffer_tl.erase (buffer_tl.begin(), buffer_tl.begin() + 3.0*tau);
				buffer_cl.erase (buffer_cl.begin(), buffer_cl.begin() + 3.0*tau);
				buffer_fl.erase (buffer_fl.begin(), buffer_fl.begin() + 3.0*tau);
				
				buffer_t1.erase (buffer_t1.begin(), buffer_t1.begin() + 3.0*tau);
				buffer_c1.erase (buffer_c1.begin(), buffer_c1.begin() + 3.0*tau);
				buffer_f1.erase (buffer_f1.begin(), buffer_f1.begin() + 3.0*tau);
				buffer_tl1.erase (buffer_tl1.begin(), buffer_tl1.begin() + 3.0*tau);
				buffer_cl1.erase (buffer_cl1.begin(), buffer_cl1.begin() + 3.0*tau);
				buffer_fl1.erase (buffer_fl1.begin(), buffer_fl1.begin() + 3.0*tau);
				
				buffer_t2.erase (buffer_t2.begin(), buffer_t2.begin() + 3.0*tau);
				buffer_c2.erase (buffer_c2.begin(), buffer_c2.begin() + 3.0*tau);
				buffer_f2.erase (buffer_f2.begin(), buffer_f2.begin() + 3.0*tau);
				buffer_tl2.erase (buffer_tl2.begin(), buffer_tl2.begin() + 3.0*tau);
				buffer_cl2.erase (buffer_cl2.begin(), buffer_cl2.begin() + 3.0*tau);
				buffer_fl2.erase (buffer_fl2.begin(), buffer_fl2.begin() + 3.0*tau);
			}

			break;

		}



		//checking


//		int c = 1;
//
//		for(unsigned int i = TR0_m; i< (BJ_m + 1); i++)
//		{
//			m_pre.at(i) = c;
//		}



/*******************************************************************************
*  MODULE 6 FORWARD MODELS OF PRE MOTOR NEURONS FOR STATE ESTIMATION
*******************************************************************************/


		//******Reflex mechanisms**********

		//in0.at(R0_fs) = preprosensor.at(R0_fs);--> compare with clipped signal "m.at(CR0_m)" or original one "postcr.at(0)"
		//in0.at(R1_fs) = preprosensor.at(R1_fs);--> compare with clipped signal "m.at(CR1_m)" or original one "postcr.at(1)"
		//in0.at(R2_fs) = preprosensor.at(R2_fs);--> compare with clipped signal "m.at(CR2_m)" or original one "postcr.at(2)"
		//in0.at(L0_fs) = preprosensor.at(L0_fs);--> compare with clipped signal "m.at(CL0_m)" or original one "postcl.at(0)"
		//in0.at(L1_fs) = preprosensor.at(L1_fs);--> compare with clipped signal "m.at(CL1_m)" or original one "postcl.at(1)"
		//in0.at(L2_fs) = preprosensor.at(L2_fs);--> compare with clipped signal "m.at(CL2_m)" or original one "postcl.at(2)"

		reflex_R_fs.at(0) = in0.at(R0_fs); //R0_fs = 19
		reflex_R_fs.at(1) = in0.at(R1_fs); //R1_fs = 20
		reflex_R_fs.at(2) = in0.at(R2_fs); //R2_fs = 21
		reflex_L_fs.at(0) = in0.at(L0_fs); //L0_fs = 22
		reflex_L_fs.at(1) = in0.at(L1_fs); //L1_fs = 23
		reflex_L_fs.at(2) = in0.at(L2_fs); //L2_fs = 24


		if(reflex_R_fs.at(0)>0&&reflex_R_fs.at(1)>0&&reflex_R_fs.at(2)>0&&reflex_L_fs.at(0)>0&&reflex_L_fs.at(1)>0&&reflex_L_fs.at(2)>0)
		{
			allfoot_off_ground++; // check if all legs of ground
		}



		if(switchon_learnweights)
		{
			//cin = 0.05;
			fmodel_fmodel_cmr_w.at(0)=1.65053;//fmodel_cmr_bias= 1.3 fmodel_cmr_w.at= 1.77;counter726cin=0.05
			fmodel_fmodel_cmr_w.at(1)=1.57495;//fmodel_cmr_bias= 1.3 fmodel_cmr_w.at= 1.77;counter2513cin=0.05
			fmodel_fmodel_cmr_w.at(2)=1.29372;//fmodel_cmr_bias= 1.3 fmodel_cmr_w.at= 1.77;counter1699cin=0.05
			fmodel_fmodel_cml_w.at(0)=1.46362;//fmodel_cml_bias= 1.3 fmodel_cml_w.at= 1.77;counter2282cin=0.05
			fmodel_fmodel_cml_w.at(1)=1.57309;//fmodel_cml_bias= 1.3 fmodel_cml_w.at= 1.77;counter2532cin=0.05
			fmodel_fmodel_cml_w.at(2)=1.46362;//fmodel_cml_bias= 1.3 fmodel_cml_w.at= 1.77;counter2250cin=0.05

		}


		//Ebd_2D
		//m_pre.at(CR0_m), m_pre.at(CR1_m), m_pre.at(CR2_m), m_pre.at(CL0_m), m_pre.at(CL1_m), m_pre.at(CL2_m)

		if(switchon_ED == true)
		{
			int delay_tau = 5;

			delay_CR0.push_back (m_pre.at(CR0_m));
			delay_CR1.push_back (m_pre.at(CR1_m));
			delay_CR2.push_back (m_pre.at(CR2_m));
			delay_CL0.push_back (m_pre.at(CL0_m));
			delay_CL1.push_back (m_pre.at(CL1_m));
			delay_CL2.push_back (m_pre.at(CL2_m));

			if((delay_CR0.size()-delay_tau)>0)
			{
				m_pre_delay.at(CR0_m) = delay_CR0[delay_CR0.size()-delay_tau-1]; // 5 time steps delay
			}
			else
			{
				m_pre_delay.at(CR0_m) = 0;
			}


			if((delay_CR1.size()-delay_tau)>0)
			{
				m_pre_delay.at(CR1_m) = delay_CR1[delay_CR1.size()-delay_tau-1]; // 5 time steps delay
			}
			else
			{
				m_pre_delay.at(CR1_m) = 0;
			}

			if((delay_CR2.size()-delay_tau)>0)
			{
				m_pre_delay.at(CR2_m) = delay_CR2[delay_CR2.size()-delay_tau-1]; // 5 time steps delay
			}
			else
			{
				m_pre_delay.at(CR2_m) = 0;
			}

			if((delay_CL0.size()-delay_tau)>0)
			{
				m_pre_delay.at(CL0_m) = delay_CL0[delay_CL0.size()-delay_tau-1]; // 5 time steps delay
			}
			else
			{
				m_pre_delay.at(CL0_m) = 0;
			}

			if((delay_CL1.size()-delay_tau)>0)
			{
				m_pre_delay.at(CL1_m) = delay_CL1[delay_CL1.size()-delay_tau-1]; // 5 time steps delay
			}
			else
			{
				m_pre_delay.at(CL1_m) = 0;
			}

			if((delay_CL2.size()-delay_tau)>0)
			{
				m_pre_delay.at(CL2_m) = delay_CL2[delay_CL2.size()-delay_tau-1]; // 5 time steps delay
			}
			else
			{
				m_pre_delay.at(CL2_m) = 0;
			}

		}

		//------------CR Loop---------//
		//-1,..,+1--> tanh

		if(global_count>100)
		{
			switch(option_fmodel)
			{
			case 1:

				for(unsigned int i=0; i<fmodel_cmr_activity.size();i++)
				{
					//Forward model of Coxa right motor signals //Recurrent single neuron
					fmodel_cmr_activity.at(i) = m_pre.at(i+CR0_m/*6*/)*fmodel_cmr_w.at(i)+fmodel_cmr_output.at(i)*fmodel_fmodel_cmr_w.at(i)+fmodel_cmr_bias.at(i);
					fmodel_cmr_output.at(i) = tanh(fmodel_cmr_activity.at(i));

					//Post processing
					fmodel_cmr_outputfinal.at(i) = tanh(fmodel_cmr_output.at(i)*fmodel_post_cmr_w.at(i));

					//Calculate error
					fmodel_cmr_error.at(i) = reflex_R_fs.at(i)-fmodel_cmr_outputfinal.at(i) /*regulate error*/; //target - output // only positive error
					acc_cmr_error_old.at(i) = fmodel_cmr_error.at(i);// for elevator reflex

					if(switchon_purefootsignal)//Only foot contact signal
					{
						fmodel_cmr_error.at(i) = reflex_R_fs.at(i)-(-1/*constant error*/); //target - output // only positive error
					}


					double error_threshold = 0.15;

					// Error threshold
					if(fmodel_cmr_error.at(i)<error_threshold)//0.05)//-1)
					{
						fmodel_cmr_error.at(i) = 0.0;

						if(softlanding)// Later use sensor to activate!!!! e.g., acc sensor "g"
						{
							if(allfoot_off_ground> 50)
							{
								acc_cmr_error.at(i) = 0.0; // reset to immediately reset as soon as no error!!!!!! GOOD WHEN DROPPING ROBOT ON FLOOR
							}
							if(allfoot_off_ground> 100)
							{
								allfoot_off_ground = 0;
							}
						}

						//acc_cmr_error_elev.at(i) = 0.0; // reset
					}

					//Positive Error signal for controlling searching reflexes
					acc_cmr_error.at(i) += abs(fmodel_cmr_error.at(i));

					//reset at swing phase
					if(m_pre.at(i+CR0_m/*6*/)>-0.7) //Reset error every Swing phase by detecting CR motor signal
						acc_cmr_error.at(i) = 0;

					//Negative Error signal for controlling elevator reflexes
					if(acc_cmr_error_old.at(i)<0)
					{
						acc_cmr_error_elev.at(i) += abs(acc_cmr_error_old.at(i));
						error_cmr_elev.at(i) = 1.0;
					}
					if(acc_cmr_error_old.at(i)>0)
					{
						acc_cmr_error_elev.at(i) = 0.0;
						error_cmr_elev.at(i) = 0.0;
					}


					//--------Stop learning process if error not appear for 500 time steps!
					//Count distance between two error peaks
					if(abs(fmodel_cmr_error.at(i)) == 0.0)
					{
						counter_cr.at(i)++;
					}
					if(abs(fmodel_cmr_error.at(i)) > error_threshold)
					{

						counter_cr.at(i) = 0;

					}
					/*1) TO DO NEED TO BE ADJUSTED THIS NUMBER "500"*/

					if(counter_cr.at(i) > 500) // 500 time steps !! need to be adaptive !! later
					{
						lr_fmodel_cr.at(i) = 0;

						//bj_output.at(0) = 1.0;//-0.05;//tr_activity.at(2);

					}


					//--------Stop learning process if error not appear for 500 time steps!
					//learning rule
					if(switchon_learnweights) // No learning
					{
						lr_fmodel_cr.at(i) = 0;
					}
					fmodel_fmodel_cmr_w.at(i)+= lr_fmodel_cr.at(i)*fmodel_cmr_error.at(i); // learn rear part

					//fmodel_cmr_bias.at(i) += lr_fmodel_cr.at(i)*0.5*fmodel_cmr_error.at(i);

					//learning delta rule with gradient descent
					//fmodel_fmodel_cmr_w.at(i) += lr_fmodel_cr.at(i)*fmodel_cmr_error.at(i)*(1-fmodel_cmr_outputfinal.at(i)*fmodel_cmr_outputfinal.at(i));
					//fmodel_cmr_w.at(i) += lr_fmodel_cr.at(i)*fmodel_cmr_error.at(i)*(1-fmodel_cmr_outputfinal.at(i)*fmodel_cmr_outputfinal.at(i));

					//std::cout<<"fmodel_fmodel_cmr_w.at("<<i<<")="<<fmodel_fmodel_cmr_w.at(i)<<";//fmodel_cmr_bias="<<" "<<fmodel_cmr_bias.at(i)<<" "<<"fmodel_cmr_w.at="<<" "<<fmodel_cmr_w.at(i)<<";counter"<<counter_cr.at(i)<<"cin="<<Control_input<< "\n";

				}

				//------------CL Loop---------//
				//-1,..,+1--> tanh
				for(unsigned int i=0; i<fmodel_cml_activity.size();i++)
				{
					//Forward model of Coxa left motor signals //Recurrent single neuron
					fmodel_cml_activity.at(i) = m_pre.at(i+CL0_m/*9*/)*fmodel_cml_w.at(i)+fmodel_cml_output.at(i)*fmodel_fmodel_cml_w.at(i)+fmodel_cml_bias.at(i);
					fmodel_cml_output.at(i) = tanh(fmodel_cml_activity.at(i));

					//Post processing
					fmodel_cml_outputfinal.at(i) = tanh(fmodel_cml_output.at(i)*fmodel_post_cml_w.at(i));

					//Calculate error

					fmodel_cml_error.at(i) = reflex_L_fs.at(i)-fmodel_cml_outputfinal.at(i) /*regulate error*/; //target - output // only positive error
					acc_cml_error_old.at(i) = fmodel_cml_error.at(i);// for elevator reflex

					if(switchon_purefootsignal)//Only foot contact signal
					{
						fmodel_cml_error.at(i) = reflex_L_fs.at(i)-(-1/*constant error*/); //target - output // only positive error
					}


					double error_threshold = 0.15;
					// Error threshold
					if(fmodel_cml_error.at(i)<error_threshold)//0.05)//-1)
					{
						fmodel_cml_error.at(i) = 0.0;

						if(softlanding)// Later use sensor to activate!!!! e.g., acc sensor "g"
						{
							if(allfoot_off_ground> 50)
							{
								acc_cml_error.at(i) = 0.0; // reset to immediately reset as soon as no error!!!!!! GOOD WHEN DROPPING ROBOT ON FLOOR
							}
							if(allfoot_off_ground> 100)
							{
								allfoot_off_ground = 0;
							}
						}
						//acc_cml_error.at(i) = 0.0;// reset to immediately reset as soon as no error!!!!!! GOOD WHEN DROPPING ROBOT ON FLOOR
						//acc_cml_error_elev.at(i) = 0.0; // reset
					}

					//Positive Error signal for controlling searching reflexes
					acc_cml_error.at(i) += abs(fmodel_cml_error.at(i));

					//reset at swing phase
					if(m_pre.at(i+CL0_m/*9*/)>-0.7) //Reset error every Swing phase by detecting CR motor signal
						acc_cml_error.at(i) = 0;

					//Negative Error signal for controlling elevator reflexes
					if(acc_cml_error_old.at(i)<0)
					{
						acc_cml_error_elev.at(i) += abs(acc_cml_error_old.at(i));
						error_cml_elev.at(i) = 1.0;
					}
					if(acc_cml_error_old.at(i)>0)
					{
						acc_cml_error_elev.at(i) = 0.0;
						error_cml_elev.at(i) = 0.0;
					}


					//--------Stop learning process if error not appear for 500 time steps!
					if(abs(fmodel_cml_error.at(i)) == 0.0)
					{
						counter_cl.at(i)++;
					}

					if(abs(fmodel_cml_error.at(i)) > error_threshold)
					{
						counter_cl.at(i) = 0;
					}

					if(counter_cl.at(i) > 500) // 500 time steps !! need to be adaptive !! later
					{
						lr_fmodel_cl.at(i) = 0;
					}

					//--------Stop learning process if error not appear for 500 time steps!


					//learning delta rule
					if(switchon_learnweights) // No learning
					{
						lr_fmodel_cl.at(i) = 0;
					}
					fmodel_fmodel_cml_w.at(i)+= lr_fmodel_cl.at(i)*fmodel_cml_error.at(i); // learn rear part

					//fmodel_cml_bias.at(i) += lr_fmodel_cl.at(i)*0.5*fmodel_cml_error.at(i);

					//learning delta rule with gradient descent
					//fmodel_fmodel_cml_w.at(i) += lr_fmodel_cl.at(i)*fmodel_cml_error.at(i)*(1-fmodel_cml_outputfinal.at(i)*fmodel_cml_outputfinal.at(i));
					//fmodel_cml_w.at(i) += lr_fmodel_cl.at(i)*fmodel_cml_error.at(i)*(1-fmodel_cml_outputfinal.at(i)*fmodel_cml_outputfinal.at(i));

					//std::cout<<"fmodel_fmodel_cml_w.at("<<i<<")="<<fmodel_fmodel_cml_w.at(i)<<";//fmodel_cml_bias="<<" "<<fmodel_cml_bias.at(i)<<" "<<"fmodel_cml_w.at="<<" "<<fmodel_cml_w.at(i)<<";counter"<<counter_cl.at(i)<<"cin="<<Control_input<< "\n";
				}

				break;

			case 2:
				for(unsigned int i=0; i<fmodel_cmr_activity.size();i++)
				{
					//Forward model of Coxa right motor signals //Recurrent single neuron
					fmodel_cmr_activity.at(i) = m_pre.at(i+CR0_m/*6*/)*fmodel_cmr_w.at(i)+fmodel_cmr_output.at(i)*fmodel_fmodel_cmr_w.at(i)+fmodel_cmr_bias.at(i);
					fmodel_cmr_output.at(i) = tanh(fmodel_cmr_activity.at(i));

					//Post processing
					fmodel_cmr_outputfinal.at(i) = tanh(fmodel_cmr_output.at(i)*fmodel_post_cmr_w.at(i));

					//Calculate error
					fmodel_cmr_error.at(i) = reflex_R_fs.at(i)-fmodel_cmr_output.at(i); //target - output // only positive error
					// Error threshold
					if(fmodel_cmr_error.at(i)<0.0)
					{
						fmodel_cmr_error.at(i) = 0.0;
					}


					//Lowpass filter
					lowpass_cmr_error_activity.at(i) = fmodel_cmr_error.at(i)*lowpass_cmr_w.at(i)+lowpass_cmr__error_output.at(i)*lowpass_lowpass_cmr_w.at(i)+lowpass_cmr_bias.at(i);
					lowpass_cmr__error_output.at(i) = sigmoid(lowpass_cmr_error_activity.at(i));
					//				lowpass_cmr_error_activity.at(i) = fmodel_cmr_error.at(i)*0.1+lowpass_cmr__error_output.at(i)*0.9;
					//				lowpass_cmr__error_output.at(i) = lowpass_cmr_error_activity.at(i);//tanh(lowpass_cmr_error_activity.at(i));


					//--------Stop learning process if error not appear for 500 time steps!
					if(abs(fmodel_cmr_error.at(i)) == 0.0)
					{
						counter_cr.at(i)++;
					}
					if(abs(fmodel_cmr_error.at(i)) > 0.0)
					{
						counter_cr.at(i) = 0;
					}
					if(counter_cr.at(i) > 500) // 500 time steps !! need to be adaptive !! later
					{
						lr_fmodel_cr.at(i) = 0;
					}

					acc_cmr_error.at(i) += abs(fmodel_cmr_error.at(i));
					//--------Stop learning process if error not appear for 500 time steps!

					//learning rule
					fmodel_fmodel_cmr_w.at(i)+= lr_fmodel_cr.at(i)*fmodel_cmr_error.at(i); // learn rear part

					//learning delta rule with gradient descent
					//fmodel_fmodel_cmr_w.at(i) += lr_fmodel_cr.at(i)*fmodel_cmr_error.at(i)*(1-fmodel_cmr_outputfinal.at(i)*fmodel_cmr_outputfinal.at(i));
					//fmodel_cmr_w.at(i) += lr_fmodel_cr.at(i)*fmodel_cmr_error.at(i)*(1-fmodel_cmr_outputfinal.at(i)*fmodel_cmr_outputfinal.at(i));

				}

				//------------CL Loop---------//
				//-1,..,+1--> tanh
				for(unsigned int i=0; i<fmodel_cml_activity.size();i++)
				{
					//Forward model of Coxa left motor signals //Recurrent single neuron
					fmodel_cml_activity.at(i) = m_pre.at(i+CL0_m/*9*/)*fmodel_cml_w.at(i)+fmodel_cml_output.at(i)*fmodel_fmodel_cml_w.at(i)+fmodel_cml_bias.at(i);
					fmodel_cml_output.at(i) = tanh(fmodel_cml_activity.at(i));

					//Post processing
					fmodel_cml_outputfinal.at(i) = tanh(fmodel_cml_output.at(i)*fmodel_post_cml_w.at(i));

					//Calculate error
					fmodel_cml_error.at(i) = reflex_L_fs.at(i)-fmodel_cml_output.at(i); //fmodel_cml_output.at(0);//fmodel_cml_outputfinal.at(0); // target - output // only positive error

					// Error threshold
					if(fmodel_cml_error.at(i)<-1)//0.05
					{
						fmodel_cml_error.at(i) = 0.0;
					}

					//Lowpass filter
					lowpass_cml_error_activity.at(i) = fmodel_cml_error.at(i)*lowpass_cml_w.at(i)+lowpass_cml__error_output.at(i)*lowpass_lowpass_cml_w.at(i)+lowpass_cml_bias.at(i);
					lowpass_cml__error_output.at(i) = tanh(lowpass_cml_error_activity.at(i));


					//--------Stop learning process if error not appear for 500 time steps!
					if(abs(fmodel_cml_error.at(i)) == 0.0)
						counter_cl.at(i)++;
					if(abs(fmodel_cml_error.at(i)) > 0.0)
						counter_cl.at(i) = 0;

					if(counter_cl.at(i) > 500) // 500 time steps !! need to be adaptive !! later
						lr_fmodel_cl.at(i) = 0;
					acc_cml_error.at(i) += abs(fmodel_cml_error.at(i));
					//--------Stop learning process if error not appear for 500 time steps!


					//learning delta rule
					fmodel_fmodel_cml_w.at(i)+= lr_fmodel_cl.at(i)*fmodel_cml_error.at(i); // learn rear part

					//learning delta rule with gradient descent
					//fmodel_fmodel_cml_w.at(i) += lr_fmodel_cl.at(i)*fmodel_cml_error.at(i)*(1-fmodel_cml_outputfinal.at(i)*fmodel_cml_outputfinal.at(i));
					//fmodel_cml_w.at(i) += lr_fmodel_cl.at(i)*fmodel_cml_error.at(i)*(1-fmodel_cml_outputfinal.at(i)*fmodel_cml_outputfinal.at(i));
				}

				break;

			case 3: // Emd_2D

				//------------CR Loop---------//
				for(unsigned int i=0; i<fcn_r.size();i++)
				{
//					a1_r.at(0)=616.057
//					a2_r.at(0)=-11.5996
//					a3_r.at(0)=627.656
//					a1_r.at(1)=5.12269e-66
//					a2_r.at(1)=-5.35422e-66
//					a3_r.at(1)=-2.49308e-65
//					a1_r.at(2)=-1.48251e-58
//					a2_r.at(2)=1.41991e-58
//					a3_r.at(2)=-2.90242e-58
//					a1_l.at(0)=-8.28008e-64
//					a2_l.at(0)=7.60086e-64
//					a3_l.at(0)=-1.58809e-63
//					a1_l.at(1)=0.309315
//					a2_l.at(1)=1.71813
//					a3_l.at(1)=1.47389
//					a1_l.at(2)=5.02269e-62
//					a2_l.at(2)=-5.19715e-62
//					a3_l.at(2)=-1.19105e-61

					fcn_r.at(i) = a1_r.at(i)+a2_r.at(i)*m_pre.at(i+CR0_m/*6 CR0_m*/)+a3_r.at(i)*m_pre_delay.at(i+CR0_m/*6 CR0_m*/); //CR0
					normxsq_r.at(i) = 1+m_pre.at(i+CR0_m/*6 CR0_m*/)*m_pre.at(i+CR0_m/*6 CR0_m*/)+m_pre_delay.at(i+CR0_m/*6 CR0_m*/)*m_pre_delay.at(i+CR0_m/*6 CR0_m*/); //CR0

					if(fcn_r.at(i)*reflex_R_fs.at(i)<0) //R0
					{
						fac_r.at(i)= -fcn_r.at(i)/normxsq_r.at(i);
						a1_r.at(i) = a1_r.at(i)+fac_r.at(i);
						a2_r.at(i) = a2_r.at(i)+fac_r.at(i)*m_pre.at(i+CR0_m/*6 CR0_m*/);
						a3_r.at(i) = a3_r.at(i)+fac_r.at(i)*m_pre_delay.at(i+CR0_m/*6 CR0_m*/);

					}
					//Sign function//
					if(fac_r.at(i)<0)
						pred_r.at(i) = -1;
					if(fac_r.at(i)>0)
						pred_r.at(i) = 1;
					if(fac_r.at(i)==0)
						pred_r.at(i) = 0;

					//Prediction values of Ctr signals
					//pred_r.at(0) = CR0, pred_r.at(1) = CR1, pred_r.at(2) = CR2
					//outFilenlc1<<m_pre.at(CR0_m)<<' '<<m_pre_delay.at(CR0_m)<<' '<<reflex_R_fs.at(0)<<' '<<fcn.at(0)<<endl;

					//std::cout<<"a1_r.at("<<i<<")="<<a1_r.at(i)<< "\n";
					//std::cout<<"a2_r.at("<<i<<")="<<a2_r.at(i)<< "\n";
					//std::cout<<"a3_r.at("<<i<<")="<<a3_r.at(i)<< "\n";
				}

				//------------CL Loop---------//
				for(unsigned int i=0; i<fcn_l.size();i++)
				{
					fcn_l.at(i) = a1_l.at(i)+a2_l.at(i)*m_pre.at(i+CL0_m/*9 CL0_m*/)+a3_l.at(i)*m_pre_delay.at(i+CL0_m/*9 CL0_m*/); //CL0
					normxsq_l.at(i) = 1+m_pre.at(i+CL0_m/*9 CL0_m*/)*m_pre.at(i+CL0_m/*9 CL0_m*/)+m_pre_delay.at(i+CL0_m/*9 CL0_m*/)*m_pre_delay.at(i+CL0_m/*9 CL0_m*/); //CL0

					if(fcn_l.at(i)*reflex_L_fs.at(i)<0) //L0
					{
						fac_l.at(i)= -fcn_l.at(i)/normxsq_l.at(i);
						a1_l.at(i) = a1_l.at(i)+fac_l.at(i);
						a2_l.at(i) = a2_l.at(i)+fac_l.at(i)*m_pre.at(i+CL0_m/*9 CL0_m*/);
						a3_l.at(i) = a3_l.at(i)+fac_l.at(i)*m_pre_delay.at(i+CL0_m/*9 CL0_m*/);

					}
					//Sign function//
					if(fac_l.at(i)<0)
						pred_l.at(i) = -1;
					if(fac_l.at(i)>0)
						pred_l.at(i) = 1;
					if(fac_l.at(i)==0)
						pred_l.at(i) = 0;

					//Prediction values of Ctr signals
					//pred_l.at(0) = CL0, pred_l.at(1) = CL1, pred_l.at(2) = CL2
					//outFilenlc1<<m_pre.at(CR0_m)<<' '<<m_pre_delay.at(CR0_m)<<' '<<reflex_R_fs.at(0)<<' '<<fcn.at(0)<<endl;
					//std::cout<<"a1_l.at("<<i<<")="<<a1_l.at(i)<< "\n";
					//std::cout<<"a2_l.at("<<i<<")="<<a2_l.at(i)<< "\n";
					//std::cout<<"a3_l.at("<<i<<")="<<a3_l.at(i)<< "\n";
				}


				break;


			}
		}
		// >> i/o operations here <<
		//outFilenlc1<<m_pre.at(CR0_m)<<' '<<reflex_R_fs.at(0)<<' '<<m_pre.at(CR1_m)<<' '<<reflex_R_fs.at(1)<<' '<<m_pre.at(CR2_m)<<' '<<reflex_R_fs.at(2)<<endl;

		//******Reflex mechanisms end******


/*******************************************************************************
*  MODULE 7 REFLEX MECHANISMS
*******************************************************************************/

		//******Motor mapping and searching reflexes*****

		//*****Motor mapping
		/*
		->TC range [MAX, MIN +70,...,-70 deg]
		->mapping: output-min_r/(max_r-min_r) = input-min/(max-min)
		*/


		//1) TC_front//->normal walking range: front legs 60 deg Max = 0.858, -10 deg Min = -0.143

		min_tc_f_nwalking = 0.0143*min_tc_f_nwalking_deg;
		max_tc_f_nwalking = 0.0143*max_tc_f_nwalking_deg;

		m_reflex.at(TR0_m) = (((m_pre.at(TR0_m)-min_tc)/(max_tc-min_tc))*(max_tc_f_nwalking-min_tc_f_nwalking))+min_tc_f_nwalking;
		m_reflex.at(TL0_m) = (((m_pre.at(TL0_m)-min_tc)/(max_tc-min_tc))*(max_tc_f_nwalking-min_tc_f_nwalking))+min_tc_f_nwalking;

		//convert from activation to deg
		m_deg.at(TR0_m) = 70*m_reflex.at(TR0_m);
		m_deg.at(TL0_m) = 70*m_reflex.at(TL0_m);

		//std::cout<<"motor deg TR0"<<":"<<m_deg.at(TR0_m)<< "\n";
		//std::cout<<"motor deg TL0"<<":"<<m_deg.at(TL0_m)<< "\n";

		//TC_middle//->normal walking range: middle legs 30 deg Max = 0.501, -40 deg Min = -0.668

		min_tc_m_nwalking = 0.0167*min_tc_m_nwalking_deg;
		max_tc_m_nwalking = 0.0167*max_tc_m_nwalking_deg;

		m_reflex.at(TR1_m) = (((m_pre.at(TR1_m)-min_tc)/(max_tc-min_tc))*(max_tc_m_nwalking-min_tc_m_nwalking))+min_tc_m_nwalking;
		m_reflex.at(TL1_m) = (((m_pre.at(TL1_m)-min_tc)/(max_tc-min_tc))*(max_tc_m_nwalking-min_tc_m_nwalking))+min_tc_m_nwalking;

		//convert from activation to deg
		m_deg.at(TR1_m) = 60*m_reflex.at(TR1_m);
		m_deg.at(TL1_m) = 60*m_reflex.at(TL1_m);

		//std::cout<<"motor deg TR1"<<":"<<m_deg.at(TR1_m)<< "\n";
		//std::cout<<"motor deg TL1"<<":"<<m_deg.at(TL1_m)<< "\n";

		//TC_rear//->normal walking range: hind legs 10 deg Max = 0.143, -60 deg Min = -0.858

		min_tc_r_nwalking = 0.0143*min_tc_r_nwalking_deg;
		max_tc_r_nwalking = 0.0143*max_tc_r_nwalking_deg;

		m_reflex.at(TR2_m) = (((m_pre.at(TR2_m)-min_tc)/(max_tc-min_tc))*(max_tc_r_nwalking-min_tc_r_nwalking))+min_tc_r_nwalking;
		m_reflex.at(TL2_m) = (((m_pre.at(TL2_m)-min_tc)/(max_tc-min_tc))*(max_tc_r_nwalking-min_tc_r_nwalking))+min_tc_r_nwalking;

		//convert from activation to deg
		m_deg.at(TR2_m) = 70*m_reflex.at(TR2_m);
		m_deg.at(TL2_m) = 70*m_reflex.at(TL2_m);

		//std::cout<<"motor deg TR2"<<":"<<m_deg.at(TR2_m)<< "\n";
		//std::cout<<"motor deg TL2"<<":"<<m_deg.at(TL2_m)<< "\n";


		//*****Motor mapping & Searching reflexes

		//2) CTr range [+75,...,-75 deg]
		//normal walking range
		//up 75 deg Max = 1.0, 50 deg Min = 0.715

		for(unsigned int i=0; i<acc_cmr_error.size();i++)
		{
			//1) positive accumulated error = acc_cmr_error.at(i) -->for searching reflexes
			//2) negative accumulated error --> for elevating reflex


			offset_ctr.at(i) = 0.0;/*0...115 Linear or Exponential function!!**/
			offset_ctl.at(i) = 0.0;/*Linear or Exponential function!!**/

			if(switchon_reflexes)
			{
				/*2) TO DO NEED TO BE ADJUSTED THIS MAPPING FROM ACC_ERROR TO OFFSET*/

				offset_ctr.at(i) = acc_cmr_error.at(i)*(max_c/max_c_offset);//0.6216;/*0...115 Linear or Exponential function!!**/
				offset_ctl.at(i) = acc_cml_error.at(i)*(max_c/max_c_offset);//0.6216;/*Linear or Exponential function!!**/

				offset_ctr.at(2) = acc_cmr_error.at(i)*(max_c/ (max_c_offset+40));//0.5946;/*0...110 Linear or Exponential function!!**/
				offset_ctl.at(2) = acc_cml_error.at(i)*(max_c/ (max_c_offset+40));//0.5946;/*0...110 Linear or Exponential function!!**/

				//max_c_offset ; Adjust from 45,...185=> 185 (cin= 0.02, wave), 45 (cin= 0.18, tripod)
			}

			//offset_ctr.at(i) = 150.0;// Change this parameter to extend leg
			//offset_ctl.at(i) = 150.0;// Change this parameter to extend leg

			//------Right CTR joints---------------------//
			//acc error upto 0...180

			//-----------Searching reflexes START---------------------------------------//
			//min_ctr_nwalking.at(i) = 0.0143*(min_ctr_nwalking_deg.at(i)-offset_ctr.at(i));//0.715;// MIN -70 deg
			//max_ctr_nwalking.at(i) = 0.0143*(max_ctr_nwalking_deg.at(i)-offset_ctr.at(i));//1; // MAX +70 deg
			min_ctr_nwalking.at(i) = 0.0133*(min_ctr_nwalking_deg.at(i)-offset_ctr.at(i));//0.715;// 50 deg, MIN -75 deg
			max_ctr_nwalking.at(i) = 0.0133*(max_ctr_nwalking_deg.at(i)-offset_ctr.at(i));//1; // 70 deg, MAX +75 deg

			m_reflex.at(i+CR0_m/*6*/) = (((m_pre.at(i+CR0_m/*6*/)-min_ctr)/(max_ctr-min_ctr))*(max_ctr_nwalking.at(i)-min_ctr_nwalking.at(i)))+min_ctr_nwalking.at(i);
			//m_reflex.at(i+CR0_m/*6*/) = 0.7865; //= +55 deg M shape with body touch ground
			//-----------Searching reflexes END----------------------------------------//

			//convert from activation to deg
			m_deg.at(i+CR0_m/*6*/) = 75*m_reflex.at(i+CR0_m/*6*/);
			//std::cout<<"motor deg CR0"<<":"<<m_deg.at(CR0_m)<< "\n";


			//------Left CTR joints---------------------//
			//acc error upto 0...180

			//-----------Searching reflexes START----------------------------------------//
			min_ctl_nwalking.at(i) = 0.0133*(min_ctl_nwalking_deg.at(i)-offset_ctl.at(i));//0.715;// 50 deg
			max_ctl_nwalking.at(i) = 0.0133*(max_ctl_nwalking_deg.at(i)-offset_ctl.at(i));//1; // 70 deg

			m_reflex.at(i+CL0_m/*9*/) = (((m_pre.at(i+CL0_m/*9*/)-min_ctr)/(max_ctr-min_ctr))*(max_ctl_nwalking.at(i)-min_ctl_nwalking.at(i)))+min_ctl_nwalking.at(i);
			//-----------Searching reflexes END-----------------------------------------//


			//convert from activation to deg
			m_deg.at(i+CL0_m/*9*/) = 75*m_reflex.at(i+CL0_m/*9*/);
			//std::cout<<"motor deg CL0"<<":"<<m_deg.at(CL0_m)<< "\n";

			//std::cout<<"motor deg CR"<<" "<<i<<":"<<m_deg.at(i+CR0_m/*6*/)<< "\n";
			//std::cout<<"motor deg CL"<<" "<<i<<":"<<m_deg.at(i+CL0_m/*6*/)<< "\n";

		}

		//3) FTi range [-20,...,-130 deg] neural activation = 0.0182*angle(deg)+1.3636
		//normal walking range
		//up -120 deg Max = -0.8204, -130 deg Min = -1.0024

		for(unsigned int i=0; i<acc_cmr_error.size();i++)
		{

			//------Right FTI joints---------------------//
			//acc error upto 0...180

			if(switchon_reflexes==true)
			{
				/*2) TO DO NEED TO BE ADJUSTED THIS MAPPING FROM ACC_ERROR TO OFFSET*/

				offset_ftir.at(i) = acc_cmr_error.at(i)*(max_f/ max_f_offset);//0.5946;/*0...110 Linear or Exponential function!!**/
				offset_ftil.at(i) = acc_cml_error.at(i)*(max_f/ max_f_offset);//0.5946;/*0...110 Linear or Exponential function!!**/

				//Too make FTI less extend for more stable walking
				offset_ftir.at(2) = acc_cmr_error.at(i)*(max_f/ (max_f_offset));//+40));//0.5946;/*0...110 Linear or Exponential function!!**/
				offset_ftil.at(2) = acc_cml_error.at(i)*(max_f/ (max_f_offset));//+40));//0.5946;/*0...110 Linear or Exponential function!!**/

				//max_f_offset = 45.0; // Adjust from 45,...185=> 185 (cin= 0.02, wave), 45 (cin= 0.18, tripod)

			}

			if(switchon_reflexes==false)
			{
				offset_ftir.at(i) = 0.0;/*0...110 Linear or Exponential function!!**/
				offset_ftil.at(i) = 0.0;/*0...110 Linear or Exponential function!!**/
				acc_cmr_error_elev.at(i) = 0.0;
				acc_cml_error_elev.at(i) = 0.0;
			}

			//offset_ftir.at(i) = 150.0;// Change this parameter to extend leg
			//offset_ftil.at(i) = 150.0;// Change this parameter to extend leg

			//-----------Searching reflexes START----------------------------------------//
			min_ftir_nwalking.at(i) = 0.0182*(min_ftir_nwalking_deg.at(i)+offset_ftir.at(i))+1.3636;
			max_ftir_nwalking.at(i) = 0.0182*(max_ftir_nwalking_deg.at(i)+offset_ftir.at(i))+1.3636;

			m_reflex.at(i+FR0_m/*12*/) = (((m_pre.at(i+FR0_m/*12*/)-min_fti)/(max_fti-min_fti))*(max_ftir_nwalking.at(i)-min_ftir_nwalking.at(i)))+min_ftir_nwalking.at(i);
			//-----------Searching reflexes END-----------------------------------------//

			//-----------Elevator reflexes START----------------------------------------//
			if(acc_cmr_error_elev.at(i)>6.5)//7.0
			{
				m_reflex.at(i+FR0_m/*12*/) = 1.0;//0.6;//0.4;
				m_reflex.at(i+CR0_m) = 1.0;

				//For TR0
				if(i==0)
				{
					m_reflex.at(i+TR0_m) = -0.2;
					for(int x=0; x<5; x++)
					{
						m_reflex.at(i+TR0_m/*12*/) = -0.1+x*0.1;
					}
				}

				//For TR1
				if(i==1)
				{
					m_reflex.at(i+TR0_m) = -0.2;
					for(int x=0; x<5; x++)
					{
						m_reflex.at(i+TR0_m/*12*/) = -0.6+x*0.1;
					}
				}

				//For TR2
				if(i==2)
				{
					m_reflex.at(i+TR0_m) = -0.2;//-0.2
					for(int x=0; x<5; x++)
					{
						m_reflex.at(i+TR0_m/*12*/) = 0.0;//-0.95+x*0.1;
					}
				}

			}

			//convert from activation to deg
			m_deg.at(i+FR0_m/*12*/) = 55*m_reflex.at(i+FR0_m/*12*/)-75;

			//-----------Elevator reflexes END----------------------------------------//

			//------Left FTI joints---------------------//
			//acc error upto 0...180

			//-----------Searching reflexes START----------------------------------------//
			min_ftil_nwalking.at(i) = 0.0182*(min_ftil_nwalking_deg.at(i)+offset_ftil.at(i))+1.3636;
			max_ftil_nwalking.at(i) = 0.0182*(max_ftil_nwalking_deg.at(i)+offset_ftil.at(i))+1.3636;

			m_reflex.at(i+FL0_m) = (((m_pre.at(i+FL0_m)-min_fti)/(max_fti-min_fti))*(max_ftil_nwalking.at(i)-min_ftil_nwalking.at(i)))+min_ftil_nwalking.at(i);
			//-----------Searching reflexes START----------------------------------------//

			//-----------Elevator reflexes START----------------------------------------//
			if(acc_cml_error_elev.at(i)>6.5)//7.0)
			{
			//if(acc_cml_error_old.at(0)<-1)
				m_reflex.at(i+FL0_m/*12*/) = 1.0;//0.6;//0.4;
				m_reflex.at(i+CL0_m) = 1.0;

				//For TL0
				if(i==0)
				{
					m_reflex.at(i+TL0_m) = -0.2;
					for(int x=0; x<5; x++)
					{
						m_reflex.at(i+TL0_m/*12*/) = -0.1+x*0.1;
					}
				}

				//For TL1
				if(i==1)
				{
					m_reflex.at(i+TL0_m) = -0.2;
					for(int x=0; x<5; x++)
					{
						m_reflex.at(i+TL0_m/*12*/) = -0.6+x*0.1;
					}
				}

				//For TL2
				if(i==2)
				{
					m_reflex.at(i+TL0_m) = -0.2;//-0.2
					for(int x=0; x<5; x++)
					{
						m_reflex.at(i+TL0_m/*12*/) = 0.0;//-0.95+x*0.1;
					}
				}
			}

			//-----------Elevator reflexes END----------------------------------------//

			//convert from activation to deg
			m_deg.at(i+FL0_m) = 55*m_reflex.at(i+FL0_m)-75;

			//m_reflex.at(i+FR0_m/*12*/) = 1; //max = -20 deg
			//m_reflex.at(i+FL0_m/*15*/) = 1; //max = -20 deg
			//m_reflex.at(i+FR0_m/*12*/) = -1; //min = -130 deg //M shape with body touch ground
			//m_reflex.at(i+FL0_m/*15*/) = -1; //min = -130 deg //M shape with body touch ground

			//std::cout<<"motor deg FR"<<" "<<i<<":"<<m_deg.at(i+FR0_m/*6*/)<< "\n";
			//std::cout<<"motor deg FL"<<" "<<i<<":"<<m_deg.at(i+FL0_m/*6*/)<< "\n";
		}

		//4) BJ range [-45,...,45 deg] neural activation = 0.0222*angle(deg)
		//normal walking range
		//=0.0

		min_bj_fwalking = 0.0222*min_bj_fwalking_deg;
		max_bj_fwalking = 0.0222*max_bj_fwalking_deg;

		m_reflex.at(BJ_m) = (((bj_output.at(0)-min_bj)/(max_bj-min_bj))*(max_bj_fwalking-min_bj_fwalking))+min_bj_fwalking;

		//convert from activation to deg
		m_deg.at(BJ_m) =  45*m_reflex.at(BJ_m);
		//m_reflex.at(BJ_m) = 1; // max 45 deg
		//m_reflex.at(BJ_m) = -1; // min -45 deg
		//std::cout<<"motor deg BJ"<<":"<<m_deg.at(BJ_m)<< "\n";


/*******************************************************************************
*  MODULE 8 MUSCLE MODELS
*******************************************************************************/
/*XIAOFENG*/

/*******************************************************************************
*  FINAL MOTOR OUTPUTS TO MOTOR NEURONS
*******************************************************************************/


		//------------------Reading signals from Text file
		if(reading_text_testing)
		{
			//Test the reading function
			char str[10];
			//std::string str;

			//Opens for reading the file
			//ifstream b_file ("ManyLegs_Trans_0.11_by_AMMC_gain=0.5_trainon0.02.txt");//( "Test1.txt" );
			//ifstream b_file ("ManyLegs_Tripod_0.18_by_AMMC_gain=0.5_trainon0.02.txt");//( "Test1.txt" );
			ifstream b_file ("ManyLegs_0.05_by_AMMC_gain=0.5_trainon0.02.txt");//( "Test1.txt" );

			//Reads one string from the file

			m_r0_t_old = m.at(TR0_m);
			m_r1_t_old = m.at(TR1_m);
			m_r2_t_old = m.at(TR2_m);
			m_l0_t_old = m.at(TL0_m);
			m_l1_t_old = m.at(TL1_m);
			m_l2_t_old = m.at(TL2_m);

			if(!initialized)
			{	i_text_loop = 0;
			while(b_file>> str) //time first column
			{

				// b_file>> str;
				m_r0_text = atof(str);//input1
				m_r0_t.at(i_text_loop) = m_r0_text;

				b_file>> str;
				m_r1_text = atof(str);//input2
				m_r1_t.at(i_text_loop) = m_r1_text;

				b_file>> str;
				m_r2_text = atof(str);//input3
				m_r2_t.at(i_text_loop) = m_r2_text;

				b_file>> str;
				m_l0_text = atof(str);//input4
				m_l0_t.at(i_text_loop) = m_l0_text;

				b_file>> str;
				m_l1_text = atof(str);//input5
				m_l1_t.at(i_text_loop) = m_l1_text;

				b_file>> str;
				m_l2_text = atof(str);//input6
				m_l2_t.at(i_text_loop) = m_l2_text;


				i_text_loop++;

				//std::cout<<"mR0 "<<m_r0_text<<"mR1 "<<m_r1_text<<"mR2 "<<m_r2_text<<"mL0 "<<m_l0_text<<"mL1 "<<m_l1_text<<"mL2 "<<m_l2_text <<" "<<i_text_loop<<"\n"<<endl;

			}
			initialized = true;
			}

			///Use motor signal from text

			if(ii<i_text_loop)
			{
				ii++;
			}
			else
			{
				ii=0;
			}

			//std::cout<<"ii "<<ii<<"i "<<i_text_loop<<"\n"<<endl;
			//std::cout<<"mR0"<<m_r0_t.at(ii)<<"mR1 "<<m_r1_t.at(ii)<<"mR2 "<<m_r2_t.at(ii)<<"mL0 "<<m_l0_t.at(ii)<<"mL1 "<<m_l1_t.at(ii)<<"mL2 "<<m_l2_t.at(ii) <<" "<<i_text_loop<<"\n"<<endl;


			m.at(TR0_m) = m_r0_t.at(ii);// m_reflex.at(TR0_m);
			m.at(TR1_m) = m_r1_t.at(ii);//m_reflex.at(TR1_m);
			m.at(TR2_m) = m_r2_t.at(ii);//m_reflex.at(TR2_m);

			m.at(TL0_m) = m_l0_t.at(ii);//m_reflex.at(TL0_m);
			m.at(TL1_m) = m_l1_t.at(ii);//m_reflex.at(TL1_m);
			m.at(TL2_m) = m_l2_t.at(ii);//m_reflex.at(TL2_m);

			double up = 0.6;

			if(m.at(TR0_m)>m_r0_t_old)
				m.at(CR0_m) = m.at(TR0_m)+0.5;
			else
				m.at(CR0_m) = up;

			if(m.at(TR1_m)>m_r1_t_old)
				m.at(CR1_m) = m.at(TR1_m)+1.5;
			else
				m.at(CR1_m) = up;

			if(m.at(TR2_m)>m_r2_t_old)
				m.at(CR2_m) = m.at(TR2_m)+1.5;
			else
				m.at(CR2_m) = up;

			if(m.at(TL0_m)>m_l0_t_old)
				m.at(CL0_m) = m.at(TL0_m)+1.5;
			else
				m.at(CL0_m) = up;

			if(m.at(TL1_m)>m_l1_t_old)
				m.at(CL1_m) = m.at(TL1_m)+1.5;
			else
				m.at(CL1_m) = up;

			if(m.at(TL2_m)>m_l2_t_old)
				m.at(CL2_m) = m.at(TL2_m)+1.5;
			else
				m.at(CL2_m) = up;

			m.at(FR0_m) = -1;//m_reflex.at(FR0_m);
			m.at(FR1_m) = -1;//m_reflex.at(FR1_m);
			m.at(FR2_m) = -1;//m_reflex.at(FR2_m);

			m.at(FL0_m) = -1;//m_reflex.at(FL0_m);
			m.at(FL1_m) = -1;//m_reflex.at(FL1_m);
			m.at(FL2_m) = -1;//m_reflex.at(FL2_m);

			m.at(BJ_m) =  m_reflex.at(BJ_m);//(m_reflex.at(TR0_m)-0.35)*5;//
	
			//------------------Reading signals from Text file
		}
		else // using normal control
		{
			//leg Right Front
			m.at(TR0_m) = m_reflex.at(TR0_m);
			m.at(CR0_m) = m_reflex.at(CR0_m);
			m.at(FR0_m) = m_reflex.at(FR0_m);
			if( true == LegDamage_flag[LEG_RF] )
			{	//fix the leg
				m.at(TR0_m) = 0;
				m.at(CR0_m) = 0.7;
				m.at(FR0_m) = -0.7;
			}


			//leg Right Middle
			m.at(TR1_m) = m_reflex.at(TR1_m);
			m.at(CR1_m) = m_reflex.at(CR1_m);
			m.at(FR1_m) = m_reflex.at(FR1_m);
			if( true == LegDamage_flag[LEG_RM] )
			{	//fix the leg
				m.at(TR1_m) = 0;
				m.at(CR1_m) = 0.7;
				m.at(FR1_m) = -0.7;
			}

			//leg Right Hind
			m.at(TR2_m) = m_reflex.at(TR2_m);
			m.at(CR2_m) = m_reflex.at(CR2_m);
			m.at(FR2_m) = m_reflex.at(FR2_m);
			if( true == LegDamage_flag[LEG_RH] )
			{	//fix the leg
				m.at(TR2_m) = 0;
				m.at(CR2_m) = 0.7;
				m.at(FR2_m) = -0.7;
			}

			//leg Left Front
			m.at(TL0_m) = m_reflex.at(TL0_m);
			m.at(CL0_m) = m_reflex.at(CL0_m);
			m.at(FL0_m) = m_reflex.at(FL0_m);
			if( true == LegDamage_flag[LEG_LF] )
			{	//fix the leg
				m.at(TL0_m) = 0;
				m.at(CL0_m) = 0.7;
				m.at(FL0_m) = -0.7;
			}

			//leg Left Middle
			m.at(TL1_m) = m_reflex.at(TL1_m);
			m.at(CL1_m) = m_reflex.at(CL1_m);
			m.at(FL1_m) = m_reflex.at(FL1_m);
			if( true == LegDamage_flag[LEG_LM] )
			{	//fix the leg
				m.at(TL1_m) = 0;
				m.at(CL1_m) = 0.7;
				m.at(FL1_m) = -0.7;
			}

			//leg Left Hind
			m.at(TL2_m) = m_reflex.at(TL2_m);
			m.at(CL2_m) = m_reflex.at(CL2_m);
			m.at(FL2_m) = m_reflex.at(FL2_m);
			if( true == LegDamage_flag[LEG_LH] )
			{	//fix the leg
				m.at(TL2_m) = 0;
				m.at(CL2_m) = 0.7;
				m.at(FL2_m) = -0.7;
			}

			m.at(BJ_m) =  m_reflex.at(BJ_m);//(m_reflex.at(TR0_m)-0.35)*5;//

			//outFilenlc1<<m.at(TR0_m)<<' '<<m.at(TR1_m)<<' '<<m.at(TR2_m)<<' '<<m.at(TL0_m)<<' '<<m.at(TL1_m)<<' '<<m.at(TL2_m)<<endl;
			//outFilenlcr<<m.at(CR0_m)<<' '<<m.at(CR1_m)<<' '<<m.at(CR2_m)<<' '<<m.at(CL0_m)<<' '<<m.at(CL1_m)<<' '<<m.at(CL2_m)<<endl;
			//outFilenlft<<m.at(FR0_m)<<' '<<m.at(FR1_m)<<' '<<m.at(FR2_m)<<' '<<m.at(FL0_m)<<' '<<m.at(FL1_m)<<' '<<m.at(FL2_m)<<endl;
		}

		/*int data = -1;

		m.at(TR0_m) = data;//m_reflex.at(TR0_m);
		m.at(TR1_m) = data;//m_reflex.at(TR1_m);
		m.at(TR2_m) = data;//m_reflex.at(TR2_m);

		m.at(TL0_m) = data;//m_reflex.at(TL0_m);
		m.at(TL1_m) = data;//m_reflex.at(TL1_m);
		m.at(TL2_m) = data;//m_reflex.at(TL2_m);

		m.at(CL0_m) = data;//m_reflex.at(CL0_m);
		m.at(CL1_m) = data;//m_reflex.at(CL1_m);
		m.at(CL2_m) = data;//m_reflex.at(CL2_m);

		m.at(CR0_m) = data;//m_reflex.at(CR0_m);
		m.at(CR1_m) = data;//m_reflex.at(CR1_m);
		m.at(CR2_m) = data;//m_reflex.at(CR2_m);


		m.at(FR0_m) = data;//m_reflex.at(FR0_m);
		m.at(FR1_m) = data;//m_reflex.at(FR1_m);
		m.at(FR2_m) = data;//m_reflex.at(FR2_m);

		m.at(FL0_m) = data;//m_reflex.at(FL0_m);
		m.at(FL1_m) = data;//m_reflex.at(FL1_m);
		m.at(FL2_m) = data;//m_reflex.at(FL2_m);

		m.at(BJ_m) =  m_reflex.at(BJ_m);//(m_reflex.at(TR0_m)-0.35)*5;//
		 */

		global_count++;
		std::cout<<"COUNTER"<<global_count<<"allfoot_off_ground="<<allfoot_off_ground<< "\n";

		if(switchon_purefootsignal)
		{
			std::cout<<"Error = ONLY foot signal"<< "\n";
		}

		return m;

};



