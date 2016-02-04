/*
 * NeuralLocomotionControlAdaptiveClimbing.cpp
 *
 *  Created on: May 8, 2012 update
 *      Author: poramate manoonpong
 */

#include "NeuralLocomotionControlAdaptiveClimbing.h"



//3) Step function of Neural locomotion control------

NeuralLocomotionControlAdaptiveClimbing::NeuralLocomotionControlAdaptiveClimbing(){

  //Save files
  outFilenlc1.open("Neurallocomotion.dat");



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


  //pCPG vector sizes
  pcpg_output.resize(2);
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

  //PSN vector sizes
  psn_activity.resize(12);
  psn_output.resize(12);

  psn_w.resize(12);
  for(unsigned int i=0; i<psn_w.size(); i++)
  {
    psn_w.at(i).resize(12);
  }
  psn_bias.resize(3);

  //VRN vector sizes
  vrn_activity.resize(14);
  vrn_output.resize(14);

  vrn_w.resize(14);
  for(unsigned int i=0; i<vrn_w.size(); i++)
  {
    vrn_w.at(i).resize(14);
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

  buffer_t.resize(96);
  buffer_c.resize(96);
  buffer_f.resize(96);

  buffer_tr.resize(96);
  buffer_tl.resize(96);
  buffer_fm.resize(96);


  m_pre.resize(19);
  m_preold.resize(19);
  m_prescale.resize(19);
  m_reflex.resize(19);
  m.resize(19);
  m_deg.resize(19);
  m_deg_test.resize(19);


  //---Reflex motor neurons
  fmodel_cmr_activity.resize(3);
  fmodel_cmr_output.resize(3);
  fmodel_cmr_output_old.resize(3);
  fmodel_cmr_error.resize(3);
  fmodel_cmr_errorW.resize(3);

  fmodel_cml_activity.resize(3);
  fmodel_cml_output.resize(3);
  fmodel_cml_output_old.resize(3);
  fmodel_cml_error.resize(3);
  fmodel_cml_errorW.resize(3);

  fmodel_cmr_outputfinal.resize(3);
  fmodel_cml_outputfinal.resize(3);

  //---Reflex foot sensors
  reflex_R_fs.resize(3);
  reflex_L_fs.resize(3);
  reflex_R_fs_old.resize(3);//KOH add after the GIT version
  reflex_L_fs_old.resize(3);//KOH add after the GIT version
  countup_reflex_R_fs.resize(3);//KOH add after the GIT version
  countdown_reflex_R_fs.resize(3);//KOH add after the GIT version
  countup_reflex_R_fs2.resize(3);//KOH add after the GIT version
  dervi_reflex_R_fs.resize(3);//KOH add after the GIT version
  dervi_reflex_L_fs.resize(3);//KOH add after the GIT version

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
  acc_cmr_error_posi_neg.resize(3);
  dervi_fmodel_cmr_output.resize(3);
  low_pass_fmodel_cmr_error_old.resize(3);
  low_pass_fmodel_cmr_error.resize(3);

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
  acc_cml_error_posi_neg.resize(3);
  dervi_fmodel_cml_output.resize(3);
  low_pass_fmodel_cml_error_old.resize(3);
  low_pass_fmodel_cml_error.resize(3);

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
  delay_CR0.resize(96);
  delay_CR1.resize(96);
  delay_CR2.resize(96);
  delay_CL0.resize(96);
  delay_CL1.resize(96);
  delay_CL2.resize(96);
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

/*
 * virtual muscle initializaitions
 */
  
  Raw_R_fs.resize(3);
  Raw_L_fs.resize(3);

  Stiff_R.resize(3);
  Stiff_L.resize(3);

  Stiff_L_Fast.resize(3);
  Stiff_L_Slow.resize(3);
  Stiff_R_Fast.resize(3);
  Stiff_R_Slow.resize(3);

  FTiStiff_R.resize(3);
  FTiStiff_L.resize(3);

  FTiStiff_L_Fast.resize(3);
  FTiStiff_L_Slow.resize(3);
  FTiStiff_R_Fast.resize(3);
  FTiStiff_R_Slow.resize(3);

  Exp_R_fs.resize(3);
  Exp_L_fs.resize(3);

  Exp_R_fs_old.resize(3);
  Exp_L_fs_old.resize(3);

  Diff_R_fs.resize(3);
  Diff_L_fs.resize(3);

  AveDiff_L_fs.resize(3);
  AveDiff_R_fs.resize(3);
  OldAveDiff_L_fs.resize(3);
  OldAveDiff_R_fs.resize(3);

  acc_Diff_R_fs.resize(3);
  acc_Diff_L_fs.resize(3);
  old_Diff_L_fs.resize(3);
  deri__Diff_L_fs.resize(3);

  SideForce.resize(3);
  SideForce_old.resize(3);


  Raw_R_fs_old.resize(3);
  Raw_L_fs_old.resize(3);
  
  m_reflex_muscle.resize(19);
  m_Prereflex.resize(19);
  
  JointAng.resize(19);
  PreJointAng.resize(19);
  
  JointVel.resize(19);
  PreJointVel.resize(19);
  


  
  Mass = 0.1;
  KneeLength = 0.075;
  HipLength = 0.115;
  Radius = 0.1;
  SampleTime = 0.01;
  bReset = 0;
  nSpeeCount = 0;
  
  iPeriodCount = 0;

  CJMaxOut = 1;
  CJMinOut = -1;
  CJMaxAng = ((-100.0/180.0)*3.14159265358979);
  CJMinAng = ((45.0/180.0)*3.14159265358979);
  
  FJMaxOut = 1;
  FJMinOut = -1;
  FJMaxAng = ((55.0/180.0)*3.14159265358979);
  FJMinAng = ((-70.0/180.0)*3.14159265358979);
  
  TouchR0 = 0;
  TouchR1 = 0;
  TouchR2 = 0;
  TouchL0 = 0;
  TouchL1 = 0;
  TouchL2 = 0;

  if (LEARNINGTYPE == 6)
  {
  dLDRLR0 = 12;
  dLDRLR1 = 12;
  dLDRLR2 = 12;
  dLDRLL0 = 12;
  dLDRLL1 = 12;
  dLDRLL2 = 12;
  }

  if (LEARNINGTYPE == 8)
  {
  dLDRLR0 = 0;
  dLDRLR1 = 0;
  dLDRLR2 = 0;
  dLDRLL0 = 0;
  dLDRLL1 = 0;
  dLDRLL2 = 0;
  }



  dSlowR0 = 0.0;
  dSlowR1= 0.0;
  dSlowR2= 0.0;
  dSlowL0= 0.0;
  dSlowL1= 0.0;
  dSlowL2= 0.0;

  dFastR0= 0.0;
  dFastR1= 0.0;
  dFastR2= 0.0;
  dFastL0= 0.0;
  dFastL1= 0.0;
  dFastL2= 0.0;

  dFTiSlowR0 = 0.0;
  dFTiSlowR1= 0.0;
  dFTiSlowR2= 0.0;
  dFTiSlowL0= 0.0;
  dFTiSlowL1= 0.0;
  dFTiSlowL2= 0.0;

  dFTiFastR0= 0.0;
  dFTiFastR1= 0.0;
  dFTiFastR2= 0.0;
  dFTiFastL0= 0.0;
  dFTiFastL1= 0.0;
  dFTiFastL2= 0.0;


  if (ISRECORD == 1)
  {

    oCTrStiff[0].open(R0CTRSTIFFFILE);
    oCTrStiff[1].open(R1CTRSTIFFFILE);
    oCTrStiff[2].open(R2CTRSTIFFFILE);
    oCTrStiff[3].open(L0CTRSTIFFFILE);
    oCTrStiff[4].open(L1CTRSTIFFFILE);
    oCTrStiff[5].open(L2CTRSTIFFFILE);
    oRealTime.open(RTIMEFILE);

  }
  dTimeDiff = 0.0;

//  FMusclActFac = 0.0;//1.0;//0.0;
//  CMusclActFac = 1.0;//1.0; //original setup for stiffness 
  
  
    FMusclActFac = 0.0;//0.1;//1.0;//1.0;//0.0;
    CMusclActFac = 1.0;//1.0; 
  
  K11 = INIFTISTIFF;//3.0;//4.0;//4.0;//3.0;
  K22 = INICTRSTIFF;//6;//7.3;//8.0;//6.0;//8.0;//6.0;
  
  D11 = 1.0;//1.0;
  D22 = 1.0;//1.0;
  
  KPlusHind = 0.0;//2.0;//2.0;
  
  K1[0] = K11;
  K1[1] = K11;
  K1[2] = K11 + KPlusHind;
  K1[3] = K11;
  K1[4] = K11;
  K1[5] = K11 + KPlusHind;
  
  K2[0] = K22;
  K2[1] = K22;
  K2[2] = K22;
  K2[3] = K22;
  K2[4] = K22;
  K2[5] = K22;
  
  if (ISRECORD == 1)
   {

     oCTrStiff[0]<<K2[0]<<'\n';
     oCTrStiff[1]<<K2[1]<<'\n';
     oCTrStiff[2]<<K2[2]<<'\n';
     oCTrStiff[3]<<K2[3]<<'\n';
     oCTrStiff[4]<<K2[4]<<'\n';
     oCTrStiff[5]<<K2[5]<<'\n';
     oRealTime<<dTimeDiff<<'\n';

   }



  //random K1 and K2

  //srand((unsigned)time(NULL));




//
//  K2[0] = 12;
//  K2[1] = 6;
//  K2[2] = 15;
//  K2[3] = 8;
//  K2[4] = 19;
//  K2[5] = 4;



  D1[0] = D11;
  D1[1] = D11;
  D1[2] = D11;
  D1[3] = D11;
  D1[4] = D11;
  D1[5] = D11;
  
  D2[0] = D22;
  D2[1] = D22;
  D2[2] = D22;
  D2[3] = D22;
  D2[4] = D22;
  D2[5] = D22;
  
  dSWFTiOffset = 0.5;
  dSWFTiFac = -0.1;

  dExErrorR0 = 0.01;
  dExErrorR1 = 0.01;
  dExErrorR2 = 0.01;
  dExErrorL0 = 0.01;
  dExErrorL1 = 0.01;
  dExErrorL2 = 0.01;
//0.0
  dGainR0 = 0.0;
  dGainR1 = 0.0;
  dGainR2 = 0.0;
  dGainL0 = 0.0;
  dGainL1 = 0.0;
  dGainL2 = 0.0;
//1000
  dAutoCorR0 = 1000;
  dAutoCorR1 = 1000;
  dAutoCorR2 = 1000;
  dAutoCorL0 = 1000;
  dAutoCorL1 = 1000;
  dAutoCorL2 = 1000;



  /*******************************************************************************
   *  Initial parameters
   *******************************************************************************/
  //---Initial parameters----//
  //---Inputs

  input.at(0) = 0;//1; --> as Python
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
  tau = 16;
  tau_l = 48;
  //time = 0;


  //Learning forward models to expected foot sensors

  for(unsigned int i=0; i<fmodel_cmr_w.size(); i++)
  {

    // MUST! fmodel_cmr_w.at(i)>fmodel_cmr_bias.at(i)
    fmodel_cmr_w.at(i) = 1.0;
    fmodel_fmodel_cmr_w.at(i) = 1.0;
    fmodel_post_cmr_w.at(i) = 50.0;
    fmodel_cmr_bias.at(i) = 1.0;

    fmodel_cml_w.at(i) = 1.0;
    fmodel_fmodel_cml_w.at(i) = 1.0;
    fmodel_post_cml_w.at(i) = 50.0;
    fmodel_cml_bias.at(i) = 1.0;

    lr_fmodel_cr.at(i) = 0.01;
    counter_cr.at(i) = 0;
    lr_fmodel_cl.at(i) = 0.01;
    counter_cl.at(i) = 0;

  }


  //-----------------------------AMOSiiV1_Config------------------------------------------//
  //W=0.04 Initial learned weights of forward models
  fmodel_fmodel_cmr_w.at(0)=1.75842; fmodel_cmr_bias.at(0)= 0.492008 ; fmodel_cmr_w.at(0)= 1.00879;//counter16690cin=0.04
  fmodel_fmodel_cmr_w.at(1)=1.37276; fmodel_cmr_bias.at(1)= 0.770924 ; fmodel_cmr_w.at(1)= 1.10033;//counter18210cin=0.04
  fmodel_fmodel_cmr_w.at(2)=1.50538; fmodel_cmr_bias.at(2)= 0.615794 ; fmodel_cmr_w.at(2)= 1.06701;//counter17634cin=0.04
  fmodel_fmodel_cml_w.at(0)=1.39489;fmodel_cml_bias.at(0)= 0.744294 ;fmodel_cml_w.at(0)= 1.082;//counter18155cin=0.04
  fmodel_fmodel_cml_w.at(1)=1.38666;fmodel_cml_bias.at(1)= 0.761661 ;fmodel_cml_w.at(1)= 1.09715;//counter18162cin=0.04
  fmodel_fmodel_cml_w.at(2)=1.22578;fmodel_cml_bias.at(2)= 0.728562 ;fmodel_cml_w.at(2)= 1.18023;//counter18389cin=0.04
  //-----------------------------AMOSiiV1_Config------------------------------------------//


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
    lowpass_lowpass_cml_w.at(i) = 5.5;
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

  //Internal model :://KOH add after the GIT version
  //case 5 counting technique
  counter_refelx_R_fs = 0;
  counter_refelx_L_fs = 0;
  max_up = 0;
  max_down=0;
  max_fmodel = 0.0;

  //-----------------------------AMOSiiV1_Config------------------------------------------//
  //Motor mapping
  //TC_front
  //Fix
  min_tc = -0.906124;//-0.91; // network output range
  max_tc = 0.900653;//0.91;// network output range
  //Adjust
  min_tc_f_nwalking_deg = -16.7;//-10.7; //deg ** MIN -45 deg
  max_tc_f_nwalking_deg = 14.7;//12.7; //deg ** MAX +45 deg
  min_tc_f_nwalking = -1;
  max_tc_f_nwalking = 1;

  //TC_middle
  //Adjust
  min_tc_m_nwalking_deg = -13.47; //deg ** MIN -45 deg
  max_tc_m_nwalking_deg = 13.47; //deg ** MAX +45 deg
  min_tc_m_nwalking = -1;
  max_tc_m_nwalking = 1;

  //TC_rear
  //Adjust
  min_tc_r_nwalking_deg = -17.0; //deg ** MIN -70 deg
  max_tc_r_nwalking_deg = 6.4; //deg ** MAX +70 deg
  min_tc_r_nwalking = -1;
  max_tc_r_nwalking = 1;


  //CTR
  //Fix
  min_ctr = -1; // network output range
  max_ctr = 0.96;// network output range

  //FTI
  //Fix
  min_fti = -0.995;//-1; // network output range
  max_fti = 0.955;//1;// network output range

  for(unsigned int i=0; i< min_ctr_nwalking_deg.size(); i++)
  {
    //Parameter set up for normal walking on flat terrain
    min_ctr_nwalking_deg.at(i) = 81;//** MIN -30 deg
    max_ctr_nwalking_deg.at(i) = 100;//** MAX +100 deg
    min_ctr_nwalking.at(i) = -1;
    max_ctr_nwalking.at(i) =1;

    min_ctl_nwalking_deg.at(i) = 81;//** MIN -30 deg
    max_ctl_nwalking_deg.at(i) = 100;//** MAX +100 deg
    min_ctl_nwalking.at(i) = -1;
    max_ctl_nwalking.at(i) = 1;

    min_ftir_nwalking_deg.at(i) = -140; //deg ** MIN -140 deg
    max_ftir_nwalking_deg.at(i) = -120; //deg ** MAX -15 deg
    min_ftir_nwalking.at(i) = -1;
    max_ftir_nwalking.at(i) = 1;

    min_ftil_nwalking_deg.at(i) = -140; //deg ** MIN -140 deg
    max_ftil_nwalking_deg.at(i)  = -120; //deg ** MAX -15 deg
    min_ftil_nwalking.at(i) = -1;
    max_ftil_nwalking.at(i) = 1;
  }


  //For relfex action
  max_c = 115.0; // max range
  max_f = 110.0; // max range

  //if  max_c_offset, max_f_offset very low = 45 then robot will extend its legs very fast--> walk very high good for rough terrain
  //if  max_c_offset, max_f_offset very large = 180 then robot will extend its legs very slow--> walk very low good for normal walking

  max_c_offset = 110;//110 (normal); 180 (lower the body)
  //60/*Use to compare*/;//80;//100.0;//55.0; // Adjust from 45,...185=> 185 (cin= 0.02, wave), 45 (cin= 0.18, tripod)
  max_f_offset = 110;//110 (normal); 180 (lower the body)
  //60/*Use to compare*/;//80;//100.0;//55.0; // Adjust from 45,...185=> 185 (cin= 0.02, wave), 45 (cin= 0.18, tripod)
  //80,100,185 == amlost the same height when using forward model and foot but different height using only foot signal

  //
  //BJ
  //Fix
  min_bj = -1; // network output range
  max_bj = 1;	// network output range

  //Adjust
  min_bj_fwalking_deg = -45; //deg ** MIN -45 deg
  max_bj_fwalking_deg = 45; //deg ** MAX 45 deg

  //-----------------------------AMOSiiV1_Config------------------------------------------//

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

  /*******************************************************************************
   *  CONTROL OPTION!!!!
   *******************************************************************************/
  //Wiring connection
  option_wiring = 2; //1==Tripod, 2==Delayed line)

  //Selecting forward models
  option_fmodel = 4;
  //1 == with threshold after fmodel & NO lowpass neuron after error;
  //2 == without threshold after fmodel & with lowpass neuron after error)
  //3 == delay embedded systems 2D
  //4 == with gradient error learning after fmodel & with lowpass neuron after error _On paper
  //5 == with simple clip function


  //Switch on delay embedded systems 2D
  switchon_ED = false;

  //Switch on or off reflexes
  switchon_reflexes = false;//1;// true==on after learning or when uses learned weights, false == off during learning

  //Switch on pure foot signal
  switchon_purefootsignal = false; // 1==on using only foot signal, 0 == using forward model & foot signal

  //Used learn weights
  switchon_learnweights = false;// 1== used learn weight, 0 == initial weights = 0.0 and let learning develops weights

  //Switch on foot inhibition
  switchon_footinhibition = false;// = hind foot inhibit front foot, false;

  //Switch on soft landing  = reset to normal walking as  soon as acc error = 0
  softlanding = false;

  //Testing controller from text (e.g. SOINN control as motor memory network)
  reading_text_testing = false;

};


NeuralLocomotionControlAdaptiveClimbing::~NeuralLocomotionControlAdaptiveClimbing(){


  //Save files
  outFilenlc1.close();

  if (ISRECORD == 1)
  {
    oCTrStiff[0].close();
    oCTrStiff[1].close();
    oCTrStiff[2].close();
    oCTrStiff[3].close();
    oCTrStiff[4].close();
    oCTrStiff[5].close();
    oRealTime.close();
  }

};
//
//MuscleOut NeuralLocomotionControlAdaptiveClimbing::VMuscles(MuscleIn Mu, JPara FJ, JPara CJ)
//{
//       
//       MuscleOut JPara;
//       
//           if (Mu.Neu < Mu.PreNeu)// && (Raw_R_fs.at(1) > FSERR)
//           {
//             
//             MusclePara ParaR11 = RungeKutta41(FJ.PrePos, FJ.PreVel, Mu.SampleTime , Mu.PreFc, 
//                 Mu.Fc, FJ.PreNeu, FJ.Neu, 0, FJ.K, FJ.D, FJ.MusLeng, 0, FJ.MusFac);
//             
//             JPara.FJPos = ParaR11.Pos;
//             JPara.FJNeu = Scaling(JPara.FJPos, FJ.MaxAng, FJ.MinAng, FJ.MaxNeu, FJ.MinNeu);
//             JPara.FJVel = ParaR11.Vel;
//                         
//             double DisVe = (FJ.MusLeng + Mu.Radius) * sin(JPara.FJPos);
//             MusclePara ParaR12 = RungeKutta42(CJ.PrePos, CJ.PreVel, Mu.SampleTime , Mu.PreFc, 
//                 Mu.Fc, CJ.PreNeu, CJ.Neu, DisVe, CJ.K, CJ.D, CJ.MusLeng, JPara.FJPos, CJ.MusFac);
//             
//             JPara.CJPos = ParaR12.Pos;
//             JPara.CJNeu = Scaling(JPara.CJPos, CJ.MaxAng, CJ.MinAng, CJ.MaxNeu, CJ.MinNeu);
//             JPara.CJVel = ParaR12.Vel;
//
//             
//           }
//           else
//           {
//             
//
//             JPara.FJNeu = FJ.Neu;
//             JPara.FJPos = Scaling(JPara.FJNeu, FJ.MaxNeu, FJ.MinNeu, FJ.MaxAng, FJ.MinAng);
//             JPara.FJVel = 0;
//             JPara.CJNeu = CJ.Neu;
//             JPara.CJPos = Scaling(JPara.CJNeu, CJ.MaxNeu, CJ.MinNeu, CJ.MaxAng, CJ.MinAng);
//             JPara.CJVel = 0;
//             
//           }
//           
//       return JPara;
//}

std::vector<double> NeuralLocomotionControlAdaptiveClimbing::step_nlc(const std::vector<double> in0, const std::vector<double> in1){//, bool Footinhibition){

  // Define local parameters//
  //std::vector<double> m;

  //Input to control locomotion
  input.at(0) = 0; //walking = 0, joint inhibition = 1
  input.at(1) = 0; //lateral = 0, no lateral motion = 1
  input.at(2) = 0; //lateral right = 0, lateral left = 1
  input.at(3) = -1; //turn left = 1,
  input.at(4) = -1; //turn right = 1,


  //		if(in1.at(BZ_acs)>200)
  //		{
  //			input.at(3) = 1; //turn left = 1,
  //			input.at(4) = -1; //turn right = 1,
  //		}
  //
  //		if(in1.at(BZ_acs)<20)
  //		{
  //			input.at(3) = -1; //turn left = 1,
  //			input.at(4) = 1; //turn right = 1,
  //		}
  //
  //		if(in1.at(BZ_acs)>20&&in1.at(BZ_acs)<200)
  //		{
  //			input.at(3) = -1; //turn left = 1,
  //			input.at(4) = -1; //turn right = 1,
  //		}



  /*******************************************************************************
   *  MODULE 1 CPG
   *******************************************************************************/


  //**********CPG***************
  //From 0.02-1.5
  //Control_input = 0.01;// slow Wave
  //Control_input = 0.02;// slow Wave
  //Control_input = 0.03;// slow Wave OK USED // with Foot inhibition Good
  //Control_input = 0.04;// fast Wave very good//for muscle model, it was always used.
  //Control_input = 0.05;//slow stable Tetrapod OK USED
  //Control_input = 0.1;
  //Control_input = 0.14; //terapod OK USED
  Control_input = 0.18; //Tripod fast OK USED // with Foot inhibition Good
  //Control_input = 0.34; //Faster than tripod

  if (iPeriodCount == 0)
  {
    gettimeofday(&StartTime, NULL);//time(&StartTime);

  }

  iPeriodCount = iPeriodCount + 1;



  if (iPeriodCount == SAVINGTESTTIME)
  {
    //Control_input = 0.01;

    if (ISSAVINGTEST == 1)
    {

      K2[0] = INICTRSTIFF;
      K2[1] = INICTRSTIFF;
      K2[2] = INICTRSTIFF;
      K2[3] = INICTRSTIFF;
      K2[4] = INICTRSTIFF;
      K2[5] = INICTRSTIFF;

    }

  }

  if (iPeriodCount == 2*SAVINGTESTTIME)
  {
    //Control_input = 0.01;

    if (ISSAVINGTEST == 1)
    {

      K2[0] = INICTRSTIFF;
      K2[1] = INICTRSTIFF;
      K2[2] = INICTRSTIFF;
      K2[3] = INICTRSTIFF;
      K2[4] = INICTRSTIFF;
      K2[5] = INICTRSTIFF;

    }

  }

  if (iPeriodCount == 3*SAVINGTESTTIME)
   {
     //Control_input = 0.01;

     if (ISSAVINGTEST == 1)
     {

       K2[0] = INICTRSTIFF;
       K2[1] = INICTRSTIFF;
       K2[2] = INICTRSTIFF;
       K2[3] = INICTRSTIFF;
       K2[4] = INICTRSTIFF;
       K2[5] = INICTRSTIFF;

     }

   }

  if (iPeriodCount > 800)
    {
      //Control_input = 0.01;
    }

  cpg_w.at(0).at(0) =  1.4;
  cpg_w.at(0).at(1) =  0.18+Control_input;//0.4;
  cpg_w.at(1).at(0) =  -0.18-Control_input;//-0.4
  cpg_w.at(1).at(1) =  1.4;

  cpg_activity.at(0) = cpg_w.at(0).at(0) * cpg_output.at(0) + cpg_w.at(0).at(1) * cpg_output.at(1) + cpg_bias;
  cpg_activity.at(1) = cpg_w.at(1).at(1) * cpg_output.at(1) + cpg_w.at(1).at(0) * cpg_output.at(0) + cpg_bias;

  for(unsigned int i=0; i<cpg_output.size();i++)
  {
    cpg_output.at(i) = tanh(cpg_activity.at(i));
  }

  //********CPG end************


  /*******************************************************************************
   *  MODULE 2 CPG POST-PROCESSING
   *******************************************************************************/

  //***CPG post processing*****

  //From CPG
  pcpg_step.at(0) = cpg_output.at(0);
  pcpg_step.at(1) = cpg_output.at(1);


  setold.at(0) = set.at(0);
  setold.at(1) = set.at(1);

  countupold.at(0) = countup.at(0);
  countupold.at(1) = countup.at(1);

  countdownold.at(0) = countdown.at(0);
  countdownold.at(1) = countdown.at(1);

  //1) Linear threshold transfer function neuron 1 , or called step function neuron//
  /********************************************************/

  if (pcpg_step.at(0)>=0.85) ////////////////////Intuitively select
  {
    set.at(0) = 1.0;
  }
  if (pcpg_step.at(0)<0.85) ////////////////////Intuitively select
  {
    set.at(0) = -1.0;
  }


  if (pcpg_step.at(1)>=0.85) ////////////////////Intuitively select
  {
    set.at(1) = 1.0;
  }
  if (pcpg_step.at(1)<0.85) ////////////////////Intuitively select
  {
    set.at(1) = -1.0;
  }


  diffset.at(0) = set.at(0)-setold.at(0); // double
  diffset.at(1) = set.at(1)-setold.at(1); // double


  //2) Count how many step of Swing
  /********************************************************/

  if (set.at(0) == 1.0)
  {
    countup.at(0) = countup.at(0)+1.0; //Delta x0 up
    countdown.at(0) = 0.0;
  }

  // Count how many step of Stance
  else if (set.at(0) == -1.0)
  {
    countdown.at(0) = countdown.at(0)+1.0; //Delta x0 down
    countup.at(0) = 0.0;
  }


  if (set.at(1) == 1.0)
  {
    countup.at(1) = countup.at(1)+1.0; //Delta x0 up
    countdown.at(1) = 0.0;
  }

  // Count how many step of Stance
  else if (set.at(1) == -1.0)
  {
    countdown.at(1) = countdown.at(1)+1.0; //Delta x0 down
    countup.at(1) = 0.0;
  }

  //3) Memorized the total steps of swing and stance
  /********************************************************/


  if (countup.at(0) == 0.0 && diffset.at(0) == -2.0 && set.at(0) == -1.0)
    deltaxup.at(0) = countupold.at(0);
  //
  if (countdown.at(0) == 0.0 && diffset.at(0) == 2.0 && set.at(0) == 1.0)
    deltaxdown.at(0) = countdownold.at(0);
  //
  //
  if (countup.at(1) == 0.0 && diffset.at(1) == -2.0 && set.at(1) == -1.0)
    deltaxup.at(1) = countupold.at(1);
  //
  if (countdown.at(1) == 0.0 && diffset.at(1) == 2.0 && set.at(1) == 1.0)
    deltaxdown.at(1) = countdownold.at(1);

  //4) Comput y up and down !!!!
  /********************************************************/

  xup.at(0) =  countup.at(0);
  xdown.at(0) = countdown.at(0);

  xup.at(1) =  countup.at(1);
  xdown.at(1) = countdown.at(1);



  ////////////Scaling Slope Up calculation////////
  yup.at(0) = ((2./deltaxup.at(0))*xup.at(0))-1;
  ////////////Scaling  Slope Down calculation//////
  ydown.at(0) = ((-2./deltaxdown.at(0))*xdown.at(0))+1;


  ////////////Scaling Slope Up calculation////////
  yup.at(1) = ((2./deltaxup.at(1))*xup.at(1))-1;
  ////////////Scaling  Slope Down calculation//////
  ydown.at(1) = ((-2./deltaxdown.at(1))*xdown.at(1))+1;


  //5) Combine y up and down !!!!
  /********************************************************/

  if (set.at(0) >= 0.0)
    pcpg_output.at(0) = yup.at(0);

  if (set.at(0) < 0.0)
    pcpg_output.at(0) = ydown.at(0);


  if (set.at(1) >= 0.0)
    pcpg_output.at(1) = yup.at(1);

  if (set.at(1) < 0.0)
    pcpg_output.at(1) = ydown.at(1);


  //********Limit upper and lower boundary

  if(pcpg_output.at(0)>1.0)
    pcpg_output.at(0) = 1.0;
  else if(pcpg_output.at(0)<-1.0)
    pcpg_output.at(0) = -1.0;

  if(pcpg_output.at(1)>1.0)
    pcpg_output.at(1) = 1.0;
  else if(pcpg_output.at(1)<-1.0)
    pcpg_output.at(1) = -1.0;

  //***CPG post processing*end*



  /*******************************************************************************
   *  MODULE 3 PSN
   *******************************************************************************/


  //**********PSN***************
  //pcpg_output.at(0) =  cpg_output.at(0);// Set signal to PSN
  //pcpg_output.at(1) =  cpg_output.at(1);//

  psn_activity.at(0) = psn_input2_w.at(0).at(0) * input.at(2) + psn_bias.at(0);
  psn_activity.at(1) = psn_input2_w.at(1).at(0) * input.at(2);

  psn_activity.at(2) = psn_pcpg_w.at(2).at(0) * pcpg_output.at(0) + psn_w.at(2).at(0) * psn_output.at(0);
  psn_activity.at(3) = psn_pcpg_w.at(3).at(1) * pcpg_output.at(1) + psn_w.at(3).at(1) * psn_output.at(1);
  psn_activity.at(4) = psn_pcpg_w.at(4).at(1) * pcpg_output.at(1) + psn_w.at(4).at(0) * psn_output.at(0);
  psn_activity.at(5) = psn_pcpg_w.at(5).at(0) * pcpg_output.at(0) + psn_w.at(5).at(1) * psn_output.at(1);

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


  /*******************************************************************************
   *  MODULE 4 VRNs
   *******************************************************************************/

  //**********VRN***************

  //VRN 1 (left)
  vrn_activity.at(0) = vrn_psn_w.at(0).at(11) * psn_output.at(11);
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

  tr_activity.at(0) = -2.5*vrn_output.at(13)+10*input.at(0);
  tr_activity.at(1) = -2.5*vrn_output.at(13)+(-10)*input.at(0);
  tr_activity.at(2) = -2.5*vrn_output.at(13)+(-10)*input.at(0);/////////////USED --> Python TR joint

  tl_activity.at(0) = -2.5*vrn_output.at(12)+10*input.at(0);
  tl_activity.at(1) = -2.5*vrn_output.at(12)+(-10)*input.at(0);
  tl_activity.at(2) = -2.5*vrn_output.at(12)+(-10)*input.at(0);/////////////USED --> Python TL joint *****


  cl_activity.at(0) = -5.0*psn_output.at(11)-1.0+10*input.at(0);
  cl_activity.at(1) =  5.0*psn_output.at(11)-1.0+10*input.at(0);
  cl_activity.at(2) = -5.0*psn_output.at(11)-1.0+10*input.at(0);

  cr_activity.at(0) =  5.0*psn_output.at(11)-1.0+10*input.at(0);
  cr_activity.at(1) = -5.0*psn_output.at(11)-1.0+10*input.at(0);
  cr_activity.at(2) =  5.0*psn_output.at(11)-0.5+10*input.at(0);/////////////USED --> Python CTR joint

  fl_activity.at(0) = -2.2*psn_output.at(10)*input.at(1)+1+10*input.at(0);//+(-2)*input.at(1);
  fl_activity.at(1) = 2.2*psn_output.at(10)*input.at(1)+1+10*input.at(0);//+(-2)*input.at(1);
  fl_activity.at(2) = -2.2*psn_output.at(10)*input.at(1)+1+10*input.at(0);//+(-2)*input.at(1);


  fr_activity.at(0) = -2.2*psn_output.at(10)*input.at(1)+1+10*input.at(0);//+(-2)*input.at(1);

  //For middle FT joint of left and right!!
  fr_activity.at(1) = 5.0*psn_output.at(10)-0.5+10*input.at(0);////////////USED

  //For Front and hind FT joint
  //fr_activity.at(2) = -5.0*psn_output.at(11)-0.5+10*input.at(0);////////////USED
  fr_activity.at(2) = 5.0*psn_output.at(11)+1+10*input.at(0);////////////USED--> Python FTi joint



  //-----CONTROL BACKBONE JOINT HERE------------------------------------------//

  bj_activity.at(0) = 0.0;//-0.05;//tr_activity.at(2); // as --> python

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

  buffer_t.push_back (tr_output.at(2)); // Not used


  //Used // Euduard Check
  buffer_c.push_back (cr_output.at(2));
  buffer_f.push_back (fr_output.at(2)); // For front and hind legs
  buffer_fm.push_back (fr_output.at(1)); // For middle leg

  buffer_tr.push_back (tr_output.at(2));//KOH Change
  buffer_tl.push_back (tl_output.at(2));//KOH Change



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



      //******Wiring with ONLY Tripod******

      m.at(TR0_m) = tr_output.at(0);
      m.at(TR1_m) = tr_output.at(1);
      m.at(TR2_m) = tr_output.at(2);

      m.at(TL0_m) = tl_output.at(0);
      m.at(TL1_m) = tl_output.at(1);
      m.at(TL2_m) = tl_output.at(2);

      m.at(CL0_m) = postcl.at(0);//cl_output.at(0);
      m.at(CL1_m) = postcl.at(1);//cl_output.at(1);
      m.at(CL2_m) = postcl.at(2);//cl_output.at(2);

      m.at(CR0_m) = postcr.at(0);//cr_output.at(0);
      m.at(CR1_m) = postcr.at(1);//cr_output.at(1);
      m.at(CR2_m) = postcr.at(2);//cr_output.at(2);


      m.at(FR0_m) = fr_output.at(0);
      m.at(FR1_m) = fr_output.at(1);
      m.at(FR2_m) = fr_output.at(2);

      m.at(FL0_m) = fl_output.at(0);
      m.at(FL1_m) = fl_output.at(1);
      m.at(FL2_m) = fl_output.at(2);

      m.at(BJ_m) = bj_output.at(0);

      break;
      //******Wiring with ONLY Tripod end*

      //-----------------------------AMOSiiV1_Config------------------------------------------//
    case 2:
      //******Wiring with Delay line*******
      for(unsigned int i=0; i<postclold.size();i++)
      {
        postcrold.at(i) = postcr.at(i);
        postclold.at(i) = postcl.at(i);
      
      }

      
      for(unsigned int i=0; i<m_pre.size();i++)
      {
        m_preold.at(i) = m_pre.at(i);
      }


      //TR-------------------------


      //KOH change
      if(buffer_tr.size() > 0)
      {
        m_pre.at(TR2_m) = buffer_tr[buffer_tr.size()-1];
      }
      else
      {
        m_pre.at(TR2_m) = 0;
      }

      if((buffer_tr.size()-tau)>0)
      {
        m_pre.at(TR1_m) = buffer_tr[buffer_tr.size()-tau-1];
      }
      else
      {
        m_pre.at(TR1_m) = 0;
      }

      if((buffer_tr.size()-2*tau)>0)
      {
        m_pre.at(TR0_m) = buffer_tr[buffer_tr.size()-(2*tau)-1];
      }
      else
      {
        m_pre.at(TR0_m) = 0;
      }



      //TL-------------------------

      if((buffer_tl.size()-tau_l)>0)
      {
        m_pre.at(TL2_m) = buffer_tl[buffer_tl.size()-tau_l-1];
      }
      else
      {
        m_pre.at(TL2_m) = 0;
      }
      if((buffer_tl.size()-tau-tau_l)>0)
      {
        m_pre.at(TL1_m) = buffer_tl[buffer_tl.size()-tau-tau_l-1];
      }
      else
      {
        m_pre.at(TL1_m) = 0;
      }
      if((buffer_tl.size()-2*tau-tau_l)>0)
      {
        m_pre.at(TL0_m) = buffer_tl[buffer_tl.size()-(2*tau)-tau_l-1];
      }
      else
      {
        m_pre.at(TL0_m) = 0;
      }

      //CR-------------------------

      //m_pre.at(CR2_m) = buffer_c[buffer_c.size()-1];//buffer_c[buffer_c.size()-(2*tau)-1];
      
      if(buffer_c.size()>0)
      {
        /*m_pre.at(CR2_m)*/ postcr.at(2) = buffer_c[buffer_c.size()-1];//buffer_c[buffer_c.size()-(2*tau)-1];
      }

      else
      {
        /*m_pre.at(CR2_m)*/ postcr.at(2) = 0;
      }
      /*******CHANGE KOH*post processing *******/
      m_pre.at(CR2_m) = postcr.at(2);
      if(postcr.at(2)<postcrold.at(2))//postcr.at(2)<threshold_c)
        m_pre.at(CR2_m) = -1;

      if (switchon_footinhibition){
        if(in0.at(L0_fs)>-0.5)
          m_pre.at(CR2_m)	= -1;
      }
      /*******CHANGE KOH*post processing *******/

    // m_prescale.at(CR2_m) = (((m_pre.at(CR2_m)-0.8)/(1-0.8))*2.0-1.0);
          


      /*******CHANGE KOH*post processing *******/


      if((buffer_c.size()-tau)>0)
      {
        /*m_pre.at(CR1_m)*/ postcr.at(1) = buffer_c[buffer_c.size()-tau-1];
      }
      else
      {
        /*m_pre.at(CR1_m)*/ postcr.at(1) = 0;
      }
      /*******CHANGE KOH*post processing *******/
      m_pre.at(CR1_m) = postcr.at(1);
      if(postcr.at(1)<postcrold.at(1))//postcr.at(1)<threshold_c)
        m_pre.at(CR1_m) = -1;

      if (switchon_footinhibition){
        if(in0.at(R2_fs)>-0.5)
          m_pre.at(CR1_m)	= -1;
      }
      /*******CHANGE KOH*post processing *******/

      if((buffer_c.size()-2*tau)>0)
      {
        /*m_pre.at(CR0_m)*/ postcr.at(0) = buffer_c[buffer_c.size()-(2*tau)-1];//buffer_c[buffer_c.size()-1];
      }
      else
      {
        /*m_pre.at(CR0_m)*/ postcr.at(0) = 0;
      }
      /*******CHANGE KOH*post processing *******/
      m_pre.at(CR0_m) = postcr.at(0);
      if(postcr.at(0)<postcrold.at(0))//postcr.at(0)<threshold_c)
        m_pre.at(CR0_m) = -1;

      if (switchon_footinhibition){
        if(in0.at(R1_fs)>-0.5)
          m_pre.at(CR0_m)	= -1;
        //std::cout<<"foot inhibition"<< "\n";
      }

      //CL-------------------------

      if((buffer_c.size()-tau_l)>0)
      {
        /*m_pre.at(CL2_m)*/ postcl.at(2)  = buffer_c[buffer_c.size()-tau_l-1];
      }
      else
      {
        /*m_pre.at(CL2_m)*/ postcl.at(2) = 0;
      }
      /*******CHANGE KOH*post processing *******/
      m_pre.at(CL2_m) = postcl.at(2);
      if(postcl.at(2)<postclold.at(2))//if(postcl.at(2)<threshold_c)
        m_pre.at(CL2_m) = -1;

      if (switchon_footinhibition){
        if(in0.at(R0_fs)>-0.5)
          m_pre.at(CL2_m)	= -1;
      }
      /*******CHANGE KOH*post processing *******/


      if((buffer_c.size()-tau-tau_l)>0)
      {
        /*m_pre.at(CL1_m)*/ postcl.at(1) = buffer_c[buffer_c.size()-tau-tau_l-1];
      }
      else
      {
        /*m_pre.at(CL1_m)*/ postcl.at(1) = 0;
      }
      /*******CHANGE KOH*post processing *******/
      m_pre.at(CL1_m) = postcl.at(1);
      if(postcl.at(1)<postclold.at(1))//if(postcl.at(1)<threshold_c)
        m_pre.at(CL1_m) = -1;

      if (switchon_footinhibition){
        if(in0.at(L2_fs)>-0.5)
          m_pre.at(CL1_m)	= -1;
      }
      /*******CHANGE KOH*post processing *******/

      if((buffer_c.size()-2*tau-tau_l)>0)
      {
        /*m_pre.at(CL0_m)*/ postcl.at(0) = buffer_c[buffer_c.size()-(2*tau)-tau_l-1];
      }
      else
      {
        /*m_pre.at(CL0_m)*/ postcl.at(0) = 0;
      }
      /*******CHANGE KOH*post processing *******/
      m_pre.at(CL0_m) = postcl.at(0);
      if(postcl.at(0)<postclold.at(0))//if(postcl.at(0)<threshold_c)
        m_pre.at(CL0_m) = -1;

      if (switchon_footinhibition){
        if(in0.at(L1_fs)>-0.5)
          m_pre.at(CL0_m)	= -1;
      }
      /*******CHANGE KOH*post processing *******/



      //FR-------------------------

      if(buffer_fm.size()>0)
      {
        
        if (PYTHONUSED == 1)
        {
          m_pre.at(FR2_m) = buffer_f[buffer_f.size()-1];// as -->Python///////////WORKING
          
        }
        else
        {
          m_pre.at(FR2_m) = buffer_fm[buffer_fm.size()-1];// original
          
        }
        
       
        //m_pre.at(FR2_m) = -1*buffer_f[buffer_f.size()-1];
        
      }
      else
      {
        m_pre.at(FR2_m) = 0;
      }

      if((buffer_fm.size()-tau)>0)
      {
        if (PYTHONUSED == 1)
        {
          m_pre.at(FR1_m) = buffer_f[buffer_f.size()-tau-1]; // as-->Python
          
        }
        else
        {
          m_pre.at(FR1_m) = buffer_fm[buffer_fm.size()-tau-1]; // original
          
        }
        
        
        
        //m_pre.at(FR1_m) = -1*buffer_f[buffer_f.size()-tau-1];
        
      }
      else
      {
        m_pre.at(FR1_m) = 0;
      }

      if((buffer_f.size()-2*tau)>0)
      {
        
        if (PYTHONUSED == 1)
        {
          m_pre.at(FR0_m) = buffer_f[buffer_f.size()-(2*tau)-1]; // as--> Python
          
        }
        else
        {
          m_pre.at(FR0_m) = buffer_fm[buffer_fm.size()-(2*tau)-1]; // Change! inverse signal // nice for all setups!!!
        }
        
        
        
        //m_pre.at(FR0_m) = -1*buffer_f[buffer_f.size()-(2*tau)-1]; // Change! inverse signal // original 
        
      }
      else
      {
        m_pre.at(FR0_m) = 0;
      }


      if((buffer_fm.size()-tau_l)>0)
      {

        if (PYTHONUSED == 1)
        {
          m_pre.at(FL2_m) = buffer_f[buffer_f.size()-tau_l-1]; // as--> Python
          
        }
        else
        {
          m_pre.at(FL2_m) = buffer_fm[buffer_fm.size()-tau_l-1]; // original
          
        }
        
        
        
        
        //m_pre.at(FL2_m) = -1*buffer_f[buffer_f.size()-tau_l-1];
      }
      else
      {
        m_pre.at(FL2_m) = 0;
      }

      if((buffer_fm.size()-tau-tau_l)>0)
      {
        if (PYTHONUSED == 1)
        {
          m_pre.at(FL1_m) = buffer_f[buffer_f.size()-tau-tau_l-1]; // as--> Python
          
        }
        else
        {
          m_pre.at(FL1_m) = buffer_fm[buffer_fm.size()-tau-tau_l-1]; //original
          
        }
        
        
        
        //m_pre.at(FL1_m) = -1*buffer_f[buffer_f.size()-tau-tau_l-1];
      }
      else
      {
        m_pre.at(FL1_m) = 0;
      }
      if((buffer_f.size()-2*tau-tau_l)>0)
      {
        
        if (PYTHONUSED == 1)
        {
          m_pre.at(FL0_m) = buffer_f[buffer_f.size()-(2*tau)-tau_l-1];// as --> Python
          
        }
        else
        {
          m_pre.at(FL0_m) = buffer_fm[buffer_fm.size()-(2*tau)-tau_l-1];// Change! inverse signal // nice for all setups!!!
          
        }
        
        
        
        //m_pre.at(FL0_m) = -1*buffer_f[buffer_f.size()-(2*tau)-tau_l-1];// Change! inverse signal // original
        
      }
      else
      {
        m_pre.at(FL0_m) = 0;
      }

      m_pre.at(BJ_m) = bj_output.at(0);

      if(buffer_t.size() % (6*tau) == 0)
      {
        buffer_t.erase (buffer_t.begin(), buffer_t.begin() + 3.0*tau);
        buffer_c.erase (buffer_c.begin(), buffer_c.begin() + 3.0*tau);
        buffer_f.erase (buffer_f.begin(), buffer_f.begin() + 3.0*tau);
      }



      if(buffer_tr.size() % (6*tau) == 0)
      {
        buffer_tr.erase (buffer_tr.begin(), buffer_tr.begin() + 3.0*tau);
      }

      if(buffer_tl.size() % (6*tau) == 0)
      {
        buffer_tl.erase (buffer_tl.begin(), buffer_tl.begin() + 3.0*tau);
      }

      if(buffer_fm.size() % (6*tau) == 0)
      {
        buffer_fm.erase (buffer_fm.begin(), buffer_fm.begin() + 3.0*tau);
      }

      break;

  }

  //-----------------------------AMOSiiV1_Config------------------------------------------//





  /*******************************************************************************
   *  MODULE 6 FORWARD MODELS OF PRE MOTOR NEURONS FOR STATE ESTIMATION
   *******************************************************************************/


  //******Reflex mechanisms**********

  //[-1 (stance) = contact, +1 (swing) = no contact]
  //in0.at(R0_fs) = preprosensor.at(R0_fs);--> compare with clipped signal "m.at(CR0_m)" or original one "postcr.at(0)"
  //in0.at(R1_fs) = preprosensor.at(R1_fs);--> compare with clipped signal "m.at(CR1_m)" or original one "postcr.at(1)"
  //in0.at(R2_fs) = preprosensor.at(R2_fs);--> compare with clipped signal "m.at(CR2_m)" or original one "postcr.at(2)"
  //in0.at(L0_fs) = preprosensor.at(L0_fs);--> compare with clipped signal "m.at(CL0_m)" or original one "postcl.at(0)"
  //in0.at(L1_fs) = preprosensor.at(L1_fs);--> compare with clipped signal "m.at(CL1_m)" or original one "postcl.at(1)"
  //in0.at(L2_fs) = preprosensor.at(L2_fs);--> compare with clipped signal "m.at(CL2_m)" or original one "postcl.at(2)"
  //std::cout<<"K11 = "<<K11<<","<<"K22 = "<<K22<<"\n";

  reflex_R_fs_old.at(0) = reflex_R_fs.at(0); //R0_fs = 19
  reflex_R_fs_old.at(1) = reflex_R_fs.at(1); //R1_fs = 20
  reflex_R_fs_old.at(2) = reflex_R_fs.at(2); //R2_fs = 21
  reflex_L_fs_old.at(0) = reflex_L_fs.at(0); //L0_fs = 22
  reflex_L_fs_old.at(1) = reflex_L_fs.at(1); //L1_fs = 23
  reflex_L_fs_old.at(2) = reflex_L_fs.at(2); //L2_fs = 24
  


  OldAveDiff_R_fs.at(0) = AveDiff_R_fs.at(0);
  OldAveDiff_R_fs.at(1) = AveDiff_R_fs.at(1);
  OldAveDiff_R_fs.at(2) = AveDiff_R_fs.at(2);
  OldAveDiff_L_fs.at(0) = AveDiff_L_fs.at(0);
  OldAveDiff_L_fs.at(1) = AveDiff_L_fs.at(1);
  OldAveDiff_L_fs.at(2) = AveDiff_L_fs.at(2);

  Raw_R_fs_old.at(0) = Raw_R_fs.at(0); //R0_fs = 19
  Raw_R_fs_old.at(1) = Raw_R_fs.at(1); //R1_fs = 20
  Raw_R_fs_old.at(2) = Raw_R_fs.at(2); //R2_fs = 21
  Raw_L_fs_old.at(0) = Raw_L_fs.at(0); //L0_fs = 22
  Raw_L_fs_old.at(1) = Raw_L_fs.at(1); //L1_fs = 23
  Raw_L_fs_old.at(2) = Raw_L_fs.at(2); //L2_fs = 24
  
  SideForce_old.at(0) = Raw_R_fs_old.at(0) + Raw_R_fs_old.at(1) + Raw_R_fs_old.at(2);

  SideForce_old.at(1) = Raw_L_fs_old.at(0) + Raw_L_fs_old.at(1) + Raw_L_fs_old.at(2);

  SideForce_old.at(2) = SideForce_old.at(1)-SideForce_old.at(0);

  Exp_R_fs_old.at(0) = Exp_R_fs.at(0); //R0_fs = 19
  Exp_R_fs_old.at(1) = Exp_R_fs.at(1); //R1_fs = 20
  Exp_R_fs_old.at(2) = Exp_R_fs.at(2); //R2_fs = 21
  Exp_L_fs_old.at(0) = Exp_L_fs.at(0); //L0_fs = 22
  Exp_L_fs_old.at(1) = Exp_L_fs.at(1); //L1_fs = 23
  Exp_L_fs_old.at(2) = Exp_L_fs.at(2); //L2_fs = 24

      m_Prereflex.at(TR0_m) = m_pre.at(TR0_m);//*1.2-0.3;
      m_Prereflex.at(TR1_m) = m_pre.at(TR1_m);//*1.2+0.2;
      m_Prereflex.at(TR2_m) = m_pre.at(TR2_m);//*1.2+0.5;

      m_Prereflex.at(TL0_m) = m_pre.at(TL0_m);//*1.2-0.3;
      m_Prereflex.at(TL1_m) = m_pre.at(TL1_m);//*1.2+0.2;
      m_Prereflex.at(TL2_m) = m_pre.at(TL2_m);//*1.2+0.5;

      m_Prereflex.at(CL0_m) = m_pre.at(CL0_m);
      m_Prereflex.at(CL1_m) = m_pre.at(CL1_m);
      m_Prereflex.at(CL2_m) = m_pre.at(CL2_m);

      m_Prereflex.at(CR0_m) = m_pre.at(CR0_m);
      m_Prereflex.at(CR1_m) = m_pre.at(CR1_m);
      m_Prereflex.at(CR2_m) = m_pre.at(CR2_m);


      m_Prereflex.at(FR0_m) = m_pre.at(FR0_m);
      m_Prereflex.at(FR1_m) = m_pre.at(FR1_m);
      m_Prereflex.at(FR2_m) = m_pre.at(FR2_m);

      m_Prereflex.at(FL0_m) = m_pre.at(FL0_m);
      m_Prereflex.at(FL1_m) = m_pre.at(FL1_m);
      m_Prereflex.at(FL2_m) = m_pre.at(FL2_m);

      m_Prereflex.at(BJ_m) =  m_pre.at(BJ_m);
  



  reflex_R_fs.at(0) = in0.at(R0_fs); //R0_fs = 19
  reflex_R_fs.at(1) = in0.at(R1_fs); //R1_fs = 20
  reflex_R_fs.at(2) = in0.at(R2_fs); //R2_fs = 21
  reflex_L_fs.at(0) = in0.at(L0_fs); //L0_fs = 22
  reflex_L_fs.at(1) = in0.at(L1_fs); //L1_fs = 23
  reflex_L_fs.at(2) = in0.at(L2_fs); //L2_fs = 24
  
  Raw_R_fs.at(0) = in1.at(R0_fs); //R0_fs = 19
  Raw_R_fs.at(1) = in1.at(R1_fs); //R1_fs = 20
  Raw_R_fs.at(2) = in1.at(R2_fs); //R2_fs = 21
  Raw_L_fs.at(0) = in1.at(L0_fs); //L0_fs = 22
  Raw_L_fs.at(1) = in1.at(L1_fs); //L1_fs = 23
  Raw_L_fs.at(2) = in1.at(L2_fs); //L2_fs = 24

  // using BodyX for evaluating AMOS speed


  SumFC = Raw_R_fs.at(0)+Raw_R_fs.at(1)+Raw_R_fs.at(2)+Raw_L_fs.at(0)+Raw_L_fs.at(1)+Raw_L_fs.at(2);
  BodyX = in1.at(BX_pos);
  BodyY = in1.at(BY_pos);
  BodyZ = in1.at(BZ_pos);

  nSpeeCount = nSpeeCount + 1;

  if (nSpeeCount == 1)
    {
      AveFC = SumFC;
      PreBodyX = BodyX;
      PreBodyY = BodyY;

    }
  else
  {
    AveFC = (AveFC * (nSpeeCount -1) + SumFC)/nSpeeCount;
  }



  SpeedTime = nSpeeCount * SampleTime;

//  if(nSpeeCount == 1)
//  {
//    AveFC = SumFC;
//    PreBodyX = BodyX;
//    PreBodyY = BodyY;
//
//
//
//    std::cout<<"K1 = "<<K11<<", K2 = "<<K22<<"\n";
//    std::cout<<"PreX = "<<PreBodyX<<", PreY = "<<PreBodyY<<"\n";
//  }
//  else
//  {
//    AveFC = (AveFC * (nSpeeCount -1) + SumFC)/nSpeeCount;
//
//  }
//

  SideForce.at(0) = Raw_R_fs.at(0) + Raw_R_fs.at(1) + Raw_R_fs.at(2);
  SideForce.at(1) = Raw_L_fs.at(0) + Raw_L_fs.at(1) + Raw_L_fs.at(2);

  SideForce.at(2) = SideForce.at(1)-SideForce.at(0);

  if(TESTSTIFF == 1)
  {
    if(SideForce.at(2) > 0)//force on left legs is larger.
    {
      if(SideForce_old.at(2) > 0)
      {
        K2[3] = K2[3] - CTRSTIFFSETP;
        K2[4] = K2[4] - CTRSTIFFSETP;
        K2[5] = K2[5] - CTRSTIFFSETP;
        std::cout<<"1. Reducing Left side:"<<K2[3]<<","<<K2[4]<<","<<K2[5]<<"\n";

      }
      else
      {
        K2[3] = K2[3] + CTRSTIFFSETP;
        K2[4] = K2[4] + CTRSTIFFSETP;
        K2[5] = K2[5] + CTRSTIFFSETP;
        std::cout<<"1. Increasing Left side:"<<K2[3]<<","<<K2[4]<<","<<K2[5]<<"\n";

      }

    }
    else
    {
      if(SideForce_old.at(2) > 0)
      {
        K2[3] = K2[3] + CTRSTIFFSETP;
        K2[4] = K2[4] + CTRSTIFFSETP;
        K2[5] = K2[5] + CTRSTIFFSETP;
        std::cout<<"2. Increasing Left side:"<<K2[3]<<","<<K2[4]<<","<<K2[5]<<"\n";

      }
      else
      {
        K2[3] = K2[3] - CTRSTIFFSETP;
        K2[4] = K2[4] - CTRSTIFFSETP;
        K2[5] = K2[5] - CTRSTIFFSETP;
        std::cout<<"2. Reducing Left side:"<<K2[3]<<","<<K2[4]<<","<<K2[5]<<"\n";

      }

    }
  }


  if (SpeedTime < VELTIME)
  {
    XSpeed = abs(BodyX - PreBodyX)/SpeedTime;
    YSpeed = abs(BodyY - PreBodyY)/SpeedTime;
    bReset = false;
  }
  else
  {
    //std::cout<<"Right side:"<<K2[0]<<","<<K2[1]<<","<<K2[2]<<"\n";
    //std::cout<<"errors on Right side:"<<dAveErrorR0<<"\n";

    std::cout<<"Left side:"<<K2[3]<<","<<K2[4]<<","<<K2[5]<<"\n";

    //std::cout<<"Average forward speed is "<< abs(BodyX - PreBodyX)/SpeedTime <<"\n";
    nSpeeCount = 0;
   // K11 = K11 + 1;
    //K22 = K22 + 2;

//    K2[3] = K2[3] + CTRSTIFFSETP;
//    K2[4] = K2[4] + CTRSTIFFSETP;
//    K2[5] = K2[5] + CTRSTIFFSETP;

//    for(int h = 0;h<6;h++)
//    {
//      K2[h] =  K2[h] + CTRSTIFFSETP;
//    }






   bReset = true;

  }






  if(reflex_R_fs.at(0)>0&&reflex_R_fs.at(1)>0&&reflex_R_fs.at(2)>0&&reflex_L_fs.at(0)>0&&reflex_L_fs.at(1)>0&&reflex_L_fs.at(2)>0)
  {
    allfoot_off_ground++; // check if all legs off ground
  }


  //-----------------------------AMOSiiV1_Config------------------------------------------//
  if(switchon_learnweights)
  {

    //W=0.18 // not R0 and L0 still not perfectly learn
    fmodel_fmodel_cmr_w.at(0)=1.98882; fmodel_cmr_bias.at(0)= 0.394041 ; fmodel_cmr_w.at(0)= 0.00311119;//counter6cin=0.18
    fmodel_fmodel_cmr_w.at(1)=1.58981; fmodel_cmr_bias.at(1)= 0.509271 ; fmodel_cmr_w.at(1)= 1.08698;//counter5316cin=0.18
    fmodel_fmodel_cmr_w.at(2)=1.58142; fmodel_cmr_bias.at(2)= 0.456242 ; fmodel_cmr_w.at(2)= 1.18578;//counter5272cin=0.18
    fmodel_fmodel_cml_w.at(0)=1.56037;fmodel_cml_bias.at(0)= 0.338962 ;fmodel_cml_w.at(0)= 0.486077;//counter19cin=0.18
    fmodel_fmodel_cml_w.at(1)=1.56586;fmodel_cml_bias.at(1)= 0.591344 ;fmodel_cml_w.at(1)= 1.15971;//counter5299cin=0.18
    fmodel_fmodel_cml_w.at(2)=1.44479;fmodel_cml_bias.at(2)= 0.502707 ;fmodel_cml_w.at(2)= 1.14505;//counter5359cin=0.18

  }
  //-----------------------------AMOSiiV1_Config------------------------------------------//

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

  if(global_count>500)//100)
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

          if(counter_cr.at(i) > 1000)//500) // 500 time steps !! need to be adaptive !! later
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

          if(counter_cl.at(i) > 1000)//500) // 500 time steps !! need to be adaptive !! later
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
         // std::cout<<"a1_l.at("<<i<<")="<<a1_l.at(i)<< "\n";
          //std::cout<<"a2_l.at("<<i<<")="<<a2_l.at(i)<< "\n";
          //std::cout<<"a3_l.at("<<i<<")="<<a3_l.at(i)<< "\n";
        }


        break;


      case 4:

        for(unsigned int i=0; i<fmodel_cmr_activity.size();i++)
        {
          double lowpass_error_gain;
          lowpass_error_gain = 0.9;

          low_pass_fmodel_cmr_error_old.at(i) = low_pass_fmodel_cmr_error.at(i);
          fmodel_cmr_output_old.at(i) = fmodel_cmr_output.at(i);

          //Forward model of Coxa right motor signals //Recurrent single neuron
          fmodel_cmr_activity.at(i) = m_pre.at(i+CR0_m/*6*/)*fmodel_cmr_w.at(i)+fmodel_cmr_output.at(i)*fmodel_fmodel_cmr_w.at(i)+fmodel_cmr_bias.at(i);
          fmodel_cmr_output.at(i) = tanh(fmodel_cmr_activity.at(i));

          //Calculate error for learning
          fmodel_cmr_errorW.at(i) = reflex_R_fs.at(i)-fmodel_cmr_output.at(i);//-fmodel_cmr_outputfinal.at(i) /*regulate error*/; //target - output // only positive error

          //Post processing
          //fmodel_cmr_outputfinal.at(i) = fmodel_cmr_output.at(i);//tanh(fmodel_cmr_output.at(i)*fmodel_post_cmr_w.at(i));
          fmodel_cmr_outputfinal.at(i) = tanh(fmodel_cmr_output.at(i)*fmodel_post_cmr_w.at(i));
          //Calculate error
          fmodel_cmr_error.at(i) = reflex_R_fs.at(i)-fmodel_cmr_outputfinal.at(i) /*regulate error*/; //target - output // only positive error



          low_pass_fmodel_cmr_error.at(i) = low_pass_fmodel_cmr_error_old.at(i)*lowpass_error_gain+(1-lowpass_error_gain)*fmodel_cmr_error.at(i);

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

          if(abs(fmodel_cmr_error.at(i)) < 0.05)
            acc_cmr_error_posi_neg.at(i) = 0.0;
          acc_cmr_error_posi_neg.at(i) += fmodel_cmr_error.at(i);


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
          double error_threshold_2 = 0.25;//0.15;//0.25;
          if(abs(low_pass_fmodel_cmr_error.at(i))<error_threshold_2)//abs(fmodel_cmr_error.at(i)) == 0.0)
          {
            counter_cr.at(i)++;
          }
          if(abs(low_pass_fmodel_cmr_error.at(i))>error_threshold_2)//abs(fmodel_cmr_error.at(i)) > error_threshold)
          {

            counter_cr.at(i) = 0;

          }
          /*1) TO DO NEED TO BE ADJUSTED THIS NUMBER "500"*/

          if(counter_cr.at(i) > 500)//1000) // 500 time steps !! need to be adaptive !! later
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

          //learning delta rule with gradient descent

          //Recurrent weight, fmodel_cmr_outputfinal.at(i)
          fmodel_fmodel_cmr_w.at(i) += (1*lr_fmodel_cr.at(i))*(fmodel_cmr_errorW.at(i)*(1-fmodel_cmr_output.at(i)*fmodel_cmr_output.at(i))*fmodel_cmr_output_old.at(i));
          //fmodel_fmodel_cmr_w.at(i) += lr_fmodel_cr.at(i)*(fmodel_cmr_error.at(i)*(1- fmodel_cmr_outputfinal.at(i)* fmodel_cmr_outputfinal.at(i))*fmodel_post_cmr_w.at(i)*(1-fmodel_cmr_output.at(i)*fmodel_cmr_output.at(i))*fmodel_cmr_output_old.at(i));


          if(fmodel_fmodel_cmr_w.at(i)<0.0)
            fmodel_fmodel_cmr_w.at(i) = 0.0;

          //fmodel_fmodel_cmr_w.at(i) += lr_fmodel_cr.at(i)*(2*fmodel_cmr_error.at(i)*(1- fmodel_cmr_outputfinal.at(i)* fmodel_cmr_outputfinal.at(i))*fmodel_cmr_output_old.at(i));



          //Input weight, fmodel_cmr_outputfinal.at(i)
          fmodel_cmr_w.at(i) += lr_fmodel_cr.at(i)*(fmodel_cmr_errorW.at(i)*(1-fmodel_cmr_output.at(i)*fmodel_cmr_output.at(i))*m_pre.at(i+CR0_m/*6*/));
          if(fmodel_cmr_w.at(i)<0.0)
            fmodel_cmr_w.at(i) = 0.0;

          //Bias
          fmodel_cmr_bias.at(i) += lr_fmodel_cr.at(i)*(fmodel_cmr_errorW.at(i)*(1-fmodel_cmr_output.at(i)*fmodel_cmr_output.at(i)));
          if(fmodel_cmr_bias.at(i)<0.0)
            fmodel_cmr_bias.at(i) = 0.0;


          //std::cout<<"fmodel_fmodel_cmr_w.at("<<i<<")="<<fmodel_fmodel_cmr_w.at(i)<<"; fmodel_cmr_bias.at("<<i<<")="<<" "<<fmodel_cmr_bias.at(i)<<" "<<"; fmodel_cmr_w.at("<<i<<")="<<" "<<fmodel_cmr_w.at(i)<<";counter"<<counter_cr.at(i)<<"cin="<<Control_input<< "\n";

        }

        //------------CL Loop---------//
        //-1,..,+1--> tanh
        for(unsigned int i=0; i<fmodel_cml_activity.size();i++)
        {

          double lowpass_error_gain;
          lowpass_error_gain = 0.9;

          low_pass_fmodel_cml_error_old.at(i) = low_pass_fmodel_cml_error.at(i);
          fmodel_cml_output_old.at(i) = fmodel_cml_output.at(i);

          //Forward model of Coxa left motor signals //Recurrent single neuron
          fmodel_cml_activity.at(i) = m_pre.at(i+CL0_m/*9*/)*fmodel_cml_w.at(i)+fmodel_cml_output.at(i)*fmodel_fmodel_cml_w.at(i)+fmodel_cml_bias.at(i);
          fmodel_cml_output.at(i) = tanh(fmodel_cml_activity.at(i));

          //Calculate error for learning
          fmodel_cml_errorW.at(i) = reflex_L_fs.at(i)-fmodel_cml_output.at(i);

          //Post processing
          fmodel_cml_outputfinal.at(i) = tanh(fmodel_cml_output.at(i)*fmodel_post_cml_w.at(i));

          //Calculate error
          fmodel_cml_error.at(i) = reflex_L_fs.at(i)-fmodel_cml_outputfinal.at(i) /*regulate error*/; //target - output // only positive error
          low_pass_fmodel_cml_error.at(i) = low_pass_fmodel_cml_error_old.at(i)*lowpass_error_gain+(1-lowpass_error_gain)*fmodel_cml_error.at(i);
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
          acc_cml_error_posi_neg.at(i) += fmodel_cml_error.at(i);

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
          double error_threshold_2 = 0.25;//0.15;//0.25;
          if(abs(low_pass_fmodel_cml_error.at(i))<error_threshold_2)//abs(fmodel_cml_error.at(i)) == 0.0)
          {
            counter_cl.at(i)++;
          }
          if(abs(low_pass_fmodel_cml_error.at(i))>error_threshold_2)
            //if(abs(fmodel_cml_error.at(i)) > error_threshold)
          {
            counter_cl.at(i) = 0;
          }

          if(counter_cl.at(i) > 500)//1000)//500) // 500 time steps !! need to be adaptive !! later
          {
            lr_fmodel_cl.at(i) = 0;
          }


          //--------Stop learning process if error not appear for 500 time steps!


          //learning delta rule
          if(switchon_learnweights) // No learning
          {
            lr_fmodel_cl.at(i) = 0;
          }

          //learning delta rule with gradient descent

          //Recurrent weight, fmodel_cmr_outputfinal.at(i)
          fmodel_fmodel_cml_w.at(i) += (1*lr_fmodel_cl.at(i))*(fmodel_cml_errorW.at(i)*(1-fmodel_cml_output.at(i)*fmodel_cml_output.at(i))*fmodel_cml_output_old.at(i));
          //fmodel_fmodel_cml_w.at(i) += lr_fmodel_cl.at(i)*(fmodel_cml_error.at(i)*(1- fmodel_cml_outputfinal.at(i)* fmodel_cml_outputfinal.at(i))*fmodel_post_cml_w.at(i)*(1-fmodel_cml_output.at(i)*fmodel_cml_output.at(i))*fmodel_cml_output_old.at(i));
          if(fmodel_fmodel_cml_w.at(i)<0.0)
            fmodel_fmodel_cml_w.at(i) = 0.0;

          //Input weight, fmodel_cmr_outputfinal.at(i)
          fmodel_cml_w.at(i) += lr_fmodel_cl.at(i)*(fmodel_cml_errorW.at(i)*(1-fmodel_cml_output.at(i)*fmodel_cml_output.at(i))*m_pre.at(i+CL0_m/*6*/));
          if(fmodel_cmr_w.at(i)<0.0)
            fmodel_cmr_w.at(i) = 0.0;

          //Bias
          fmodel_cml_bias.at(i) += lr_fmodel_cl.at(i)*(fmodel_cml_errorW.at(i)*(1-fmodel_cml_output.at(i)*fmodel_cml_output.at(i)));
          if(fmodel_cml_bias.at(i)<0.0)
            fmodel_cml_bias.at(i) = 0.0;

          //std::cout<<"fmodel_fmodel_cml_w.at("<<i<<")="<<fmodel_fmodel_cml_w.at(i)<<";fmodel_cml_bias.at("<<i<<")="<<" "<<fmodel_cml_bias.at(i)<<" "<<";fmodel_cml_w.at("<<i<<")="<<" "<<fmodel_cml_w.at(i)<<";counter"<<counter_cl.at(i)<<"cin="<<Control_input<< "\n";
        }

        break;



      case 5:

        for(unsigned int i=0; i<fmodel_cmr_activity.size();i++)
        {




          //learning phase start
          if(counter_refelx_R_fs<1000)
          {
            dervi_reflex_R_fs.at(i) = reflex_R_fs.at(i)-reflex_R_fs_old.at(i);
            fmodel_cmr_output_old.at(i) = fmodel_cmr_output.at(i);

            //Post processing
            if(dervi_reflex_R_fs.at(i)>1)
            {
              fmodel_cmr_output.at(i) = postcr.at(i);
            }


            if (reflex_R_fs.at(i) > 0.0)
            {
              countup_reflex_R_fs.at(i) = countup_reflex_R_fs.at(i)+1.0; //Delta x0 up
              countdown_reflex_R_fs.at(i) = 0.0;
            }

            // Count how many step of Stance
            else if (reflex_R_fs.at(i) < 0.0)
            {
              countdown_reflex_R_fs.at(i) = countdown_reflex_R_fs.at(i)+1.0; //Delta x0 up
              countup_reflex_R_fs.at(i) = 0.0;
            }


            if(countup_reflex_R_fs.at(i)>max_up)
              max_up=countup_reflex_R_fs.at(i);
            if(countdown_reflex_R_fs.at(i)>max_down)
              max_down=countdown_reflex_R_fs.at(i);

            //						if(fmodel_cmr_output.at(i)<max_fmodel)
            //							max_fmodel = fmodel_cmr_output.at(i);

            dervi_fmodel_cmr_output.at(i) = fmodel_cmr_output.at(i) - fmodel_cmr_output_old.at(i);

            counter_refelx_R_fs++;



          }


          //Producing transformed signal
          if(postcr.at(i)>-0.119 && countup_reflex_R_fs2.at(i) < max_up)
          {
            fmodel_cmr_outputfinal.at(i) = 1.0;
            countup_reflex_R_fs2.at(i)++;
          }
          else
          {
            fmodel_cmr_outputfinal.at(i) = -1.0;
            countup_reflex_R_fs2.at(i) = 0;
          }



          //learning phase end


          //
          //Forward model of Coxa right motor signals //Recurrent single neuron
          //fmodel_cmr_activity.at(i) = postcr.at(i);//m_pre.at(i+CR0_m/*6*/)*fmodel_cmr_w.at(i)+fmodel_cmr_output.at(i)*fmodel_fmodel_cmr_w.at(i)+fmodel_cmr_bias.at(i);
          //fmodel_cmr_output.at(i) = postcr.at(i);




          //fmodel_cmr_outputfinal.at(i) = tanh(fmodel_cmr_output.at(i)*fmodel_post_cmr_w.at(i));

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
            //fmodel_cmr_error.at(i) = 0.0;

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

          if(abs(fmodel_cmr_error.at(i)) < 0.05)
            acc_cmr_error_posi_neg.at(i) = 0.0;
          acc_cmr_error_posi_neg.at(i) += fmodel_cmr_error.at(i);

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

          if(counter_cr.at(i) > 1000) // 500 time steps !! need to be adaptive !! later
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

          //learning delta rule with gradient descent

          //Recurrent weight, fmodel_cmr_outputfinal.at(i)
          //fmodel_fmodel_cmr_w.at(i) += lr_fmodel_cr.at(i)*(-2*fmodel_cmr_error.at(i)*(1-fmodel_cmr_output.at(i)*fmodel_cmr_output.at(i))*fmodel_cmr_output_old.at(i));
          //fmodel_fmodel_cmr_w.at(i) += lr_fmodel_cr.at(i)*(2*fmodel_cmr_error.at(i)*(1- fmodel_cmr_outputfinal.at(i)* fmodel_cmr_outputfinal.at(i))*fmodel_cmr_output_old.at(i));
          fmodel_fmodel_cmr_w.at(i) += lr_fmodel_cr.at(i)*(2*fmodel_cmr_error.at(i)*(1- fmodel_cmr_outputfinal.at(i)* fmodel_cmr_outputfinal.at(i))*fmodel_post_cmr_w.at(i)*(1-fmodel_cmr_output.at(i)*fmodel_cmr_output.at(i))*fmodel_cmr_output_old.at(i));



          //Input weight, fmodel_cmr_outputfinal.at(i)
          //fmodel_cmr_w.at(i) += lr_fmodel_cr.at(i)*(2*fmodel_cmr_error.at(i)*(1-fmodel_cmr_output.at(i)*fmodel_cmr_output.at(i))*m_pre.at(i+CR0_m/*6*/));

          //Bias
          fmodel_cmr_bias.at(i) += lr_fmodel_cr.at(i)*(2*fmodel_cmr_error.at(i)*(1-fmodel_cmr_output.at(i)*fmodel_cmr_output.at(i)));


          //std::cout<<counter_refelx_R_fs<<"fmodel_fmodel_cmr_w.at("<<i<<")="<<fmodel_fmodel_cmr_w.at(i)<<";//fmodel_cmr_bias="<<" "<<fmodel_cmr_bias.at(i)<<" "<<"fmodel_cmr_w.at="<<" "<<fmodel_cmr_w.at(i)<<";counter"<<counter_cr.at(i)<<"cin="<<Control_input<< "\n";

        }

        //------------CL Loop---------//
        //-1,..,+1--> tanh
        for(unsigned int i=0; i<fmodel_cml_activity.size();i++)
        {

          fmodel_cml_output_old.at(i) = fmodel_cml_output.at(i);

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
            //fmodel_cml_error.at(i) = 0.0;

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
          acc_cml_error_posi_neg.at(i) += fmodel_cml_error.at(i);

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

          if(counter_cl.at(i) > 1000)//500) // 500 time steps !! need to be adaptive !! later
          {
            lr_fmodel_cl.at(i) = 0;
          }

          //--------Stop learning process if error not appear for 500 time steps!


          //learning delta rule
          if(switchon_learnweights) // No learning
          {
            lr_fmodel_cl.at(i) = 0;
          }

          //learning delta rule with gradient descent

          //Recurrent weight, fmodel_cmr_outputfinal.at(i)
          //fmodel_fmodel_cml_w.at(i) += lr_fmodel_cl.at(i)*(-2*fmodel_cml_error.at(i)*(1-fmodel_cml_output.at(i)*fmodel_cml_output.at(i))*fmodel_cml_output_old.at(i));
          fmodel_fmodel_cml_w.at(i) += lr_fmodel_cl.at(i)*(2*fmodel_cml_error.at(i)*(1- fmodel_cml_outputfinal.at(i)* fmodel_cml_outputfinal.at(i))*fmodel_post_cml_w.at(i)*(1-fmodel_cml_output.at(i)*fmodel_cml_output.at(i))*fmodel_cml_output_old.at(i));


          //Input weight, fmodel_cmr_outputfinal.at(i)
          //fmodel_cml_w.at(i) += lr_fmodel_cl.at(i)*(2*fmodel_cml_error.at(i)*(1-fmodel_cml_output.at(i)*fmodel_cml_output.at(i))*m_pre.at(i+CL0_m/*6*/));

          //Bias
          //fmodel_cml_bias.at(i) += lr_fmodel_cl.at(i)*(2*fmodel_cml_error.at(i)*(1-fmodel_cml_output.at(i)*fmodel_cml_output.at(i)));




          //std::cout<<"fmodel_fmodel_cml_w.at("<<i<<")="<<fmodel_fmodel_cml_w.at(i)<<";//fmodel_cml_bias="<<" "<<fmodel_cml_bias.at(i)<<" "<<"fmodel_cml_w.at="<<" "<<fmodel_cml_w.at(i)<<";counter"<<counter_cl.at(i)<<"cin="<<Control_input<< "\n";
        }

        break;

    }
  }
  // >> i/o operations here <<
  //outFilenlc1<<m_pre.at(CR0_m)<<' '<<reflex_R_fs.at(0)<<' '<<low_pass_fmodel_cmr_error.at(0)<<' '<<m_pre.at(CR1_m)<<' '<<reflex_R_fs.at(1)<<' '<<low_pass_fmodel_cmr_error.at(1)<<' '<<m_pre.at(CR2_m)<<' '<<reflex_R_fs.at(2)<<' '<<low_pass_fmodel_cmr_error.at(2)<<endl;


  //******Reflex mechanisms end******

  //-----------------------------AMOSiiV1_Config------------------------------------------//

  /*******************************************************************************
   *  MODULE 7 REFLEX MECHANISMS NEED to be adjusted to MAX MIN angles for different machines
   *******************************************************************************/

  //******Motor mapping and searching reflexes*****

  //*****Motor mapping
  /*
		->TC range [MAX, MIN +45,...,-45 deg] => y = 0.0222x; y = angle deg [-45,.., 45 deg] , x = neural activation [-1,..,+1]
		->mapping: output-min_r/(max_r-min_r) = input-min/(max-min)
   */

  //1) TC_front//->normal walking range: front legs 12.7 deg Max = 0.28194, -10.7 deg Min = -0.23754

  min_tc_f_nwalking = 0.0222*min_tc_f_nwalking_deg;
  max_tc_f_nwalking = 0.0222*max_tc_f_nwalking_deg;

  m_reflex.at(TR0_m) = (((m_pre.at(TR0_m)-min_tc)/(max_tc-min_tc))*(max_tc_f_nwalking-min_tc_f_nwalking))+min_tc_f_nwalking;
  m_reflex.at(TL0_m) = (((m_pre.at(TL0_m)-min_tc)/(max_tc-min_tc))*(max_tc_f_nwalking-min_tc_f_nwalking))+min_tc_f_nwalking;

  //convert from activation to deg => x = 45*y; y = angle deg [-45,.., 45 deg] , x = neural activation [-1,..,+1]
  m_deg.at(TR0_m) = 45*m_reflex.at(TR0_m);
  m_deg.at(TL0_m) = 45*m_reflex.at(TL0_m);

  //std::cout<<"motor deg TR0"<<":"<<m_deg.at(TR0_m)<< "\n";
  //std::cout<<"motor deg TL0"<<":"<<m_deg.at(TL0_m)<< "\n";

  //TC_middle//->normal walking range: middle legs13.88 deg Max = 0.308136, -13.47 deg Min = -0.299034

  min_tc_m_nwalking = 0.0222*min_tc_m_nwalking_deg;
  max_tc_m_nwalking = 0.0222*max_tc_m_nwalking_deg;

  m_reflex.at(TR1_m) = (((m_pre.at(TR1_m)-min_tc)/(max_tc-min_tc))*(max_tc_m_nwalking-min_tc_m_nwalking))+min_tc_m_nwalking;
  m_reflex.at(TL1_m) = (((m_pre.at(TL1_m)-min_tc)/(max_tc-min_tc))*(max_tc_m_nwalking-min_tc_m_nwalking))+min_tc_m_nwalking;

  //convert from activation to deg
  m_deg.at(TR1_m) = 45*m_reflex.at(TR1_m);
  m_deg.at(TL1_m) = 45*m_reflex.at(TL1_m);

  //std::cout<<"motor deg TR1"<<":"<<m_deg.at(TR1_m)<< "\n";
  //std::cout<<"motor deg TL1"<<":"<<m_deg.at(TL1_m)<< "\n";

  //TC_rear//->normal walking range: hind legs 6.4 deg Max = 0.14208, -17 deg Min = -0.3774

  min_tc_r_nwalking = 0.0222*min_tc_r_nwalking_deg;
  max_tc_r_nwalking = 0.0222*max_tc_r_nwalking_deg;

  m_reflex.at(TR2_m) = (((m_pre.at(TR2_m)-min_tc)/(max_tc-min_tc))*(max_tc_r_nwalking-min_tc_r_nwalking))+min_tc_r_nwalking;
  m_reflex.at(TL2_m) = (((m_pre.at(TL2_m)-min_tc)/(max_tc-min_tc))*(max_tc_r_nwalking-min_tc_r_nwalking))+min_tc_r_nwalking;

  //convert from activation to deg
  m_deg.at(TR2_m) = 45*m_reflex.at(TR2_m);
  m_deg.at(TL2_m) = 45*m_reflex.at(TL2_m);

  //std::cout<<"motor deg TR2"<<":"<<m_deg.at(TR2_m)<< "\n";
  //std::cout<<"motor deg TL2"<<":"<<m_deg.at(TL2_m)<< "\n";


  //*****Motor mapping & Searching reflexes

  //2) CTr range [+100,...,-30 deg]
  //normal walking range
  //up 100 deg Max = 1.0, 80 deg Min = 0.715

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
    // CTR_front, middle, hind//->normal walking range: 100 deg Max = 1.0, 80 deg Min = 0.6935
    //Converting from deg to neural activation
    min_ctr_nwalking.at(i) = 0.0154*(min_ctr_nwalking_deg.at(i)-offset_ctr.at(i))-0.5385;//normal walking MIN 80 deg
    max_ctr_nwalking.at(i) =  0.0154*(max_ctr_nwalking_deg.at(i)-offset_ctr.at(i))-0.5385;//normal walking MAX +100 deg

    m_reflex.at(i+CR0_m/*6*/) = (((m_pre.at(i+CR0_m/*6*/)-min_ctr)/(max_ctr-min_ctr))*(max_ctr_nwalking.at(i)-min_ctr_nwalking.at(i)))+min_ctr_nwalking.at(i);
    //-----------Searching reflexes END----------------------------------------//

    //convert from activation to deg
    m_deg.at(i+CR0_m/*6*/) = 65*m_reflex.at(i+CR0_m/*6*/)+35;
    //std::cout<<"motor deg CR0"<<":"<<m_deg.at(CR0_m)<< "\n";


    //------Left CTR joints---------------------//
    //acc error upto 0...180

    //-----------Searching reflexes START----------------------------------------//
    min_ctl_nwalking.at(i) = 0.0154*(min_ctl_nwalking_deg.at(i)-offset_ctl.at(i))-0.5385;//normal walking MIN 80 deg
    max_ctl_nwalking.at(i) = 0.0154*(max_ctl_nwalking_deg.at(i)-offset_ctl.at(i))-0.5385;//normal walking MAX +100 deg

    m_reflex.at(i+CL0_m/*9*/) = (((m_pre.at(i+CL0_m/*9*/)-min_ctr)/(max_ctr-min_ctr))*(max_ctl_nwalking.at(i)-min_ctl_nwalking.at(i)))+min_ctl_nwalking.at(i);
    //-----------Searching reflexes END-----------------------------------------//


    //convert from activation to deg
    m_deg.at(i+CL0_m/*9*/) = 65*m_reflex.at(i+CL0_m/*9*/)+35;
    //std::cout<<"motor deg CL0"<<":"<<m_deg.at(CL0_m)<< "\n";

    //std::cout<<"motor deg CR"<<" "<<i<<":"<<m_deg.at(i+CR0_m/*6*/)<< "\n";
    //std::cout<<"motor deg CL"<<" "<<i<<":"<<m_deg.at(i+CL0_m/*6*/)<< "\n";

  }

  //3) FTi range [-15,...,-140 deg] neural activation = 0.016*angle(deg)+1.24
  //normal walking range
  //up -140 deg Max = -0.8204, -132 deg Min = -1.0024

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
    min_ftir_nwalking.at(i) = 0.016*(min_ftir_nwalking_deg.at(i)+offset_ftir.at(i))+1.24;
    max_ftir_nwalking.at(i) = 0.016*(max_ftir_nwalking_deg.at(i)+offset_ftir.at(i))+1.24;


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
    m_deg.at(i+FR0_m/*12*/) = 62.5*m_reflex.at(i+FR0_m/*12*/)-77.5;

    //-----------Elevator reflexes END----------------------------------------//

    //------Left FTI joints---------------------//
    //acc error upto 0...180

    //-----------Searching reflexes START----------------------------------------//
    min_ftil_nwalking.at(i) = 0.016*(min_ftil_nwalking_deg.at(i)+offset_ftil.at(i))+1.24;
    max_ftil_nwalking.at(i) = 0.016*(max_ftil_nwalking_deg.at(i)+offset_ftil.at(i))+1.24;

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
    m_deg.at(i+FL0_m) = 62.5*m_reflex.at(i+FL0_m)-77.5;

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

  //outFilenlc1<<m.at(TR0_m)<<' '<<m.at(TR1_m)<<' '<<m.at(TR2_m)<<' '<<m.at(TL0_m)<<' '<<m.at(TL1_m)<<' '<<m.at(TL2_m)<<' '<<in0.at(R2_fs)<<' '<<in0.at(R1_fs)<<' '<<in0.at(R0_fs)<<' '<<in0.at(L2_fs)<<' '<<in0.at(L1_fs)<<' '<<in0.at(L0_fs)<<endl;

  //-----------------------------AMOSiiV1_Config------------------------------------------//

  /*******************************************************************************
   *  MODULE 8 MUSCLE MODELS
   *******************************************************************************/
  /*XIAOFENG*/
  
//  MuscleIn R0In = {m_reflex.at(TR0_m),m_Prereflex.at(TR0_m),SampleTime,Raw_R_fs.at(0),Raw_R_fs_old.at(0),Radius};
//  
//  JPara FR0 = {PreJointAng.at(FR0_m), PreJointVel.at(FR0_m),m_Prereflex.at(FR0_m), m_reflex.at(FR0_m), K1,
//               D1,FMusclActFac, HipLength, FJMaxOut, FJMinOut, FJMaxAng, FJMinAng};
//  
//  JPara CR0 = {PreJointAng.at(CR0_m), PreJointVel.at(CR0_m),m_Prereflex.at(CR0_m), m_reflex.at(CR0_m), K2,
//               D2,CMusclActFac, KneeLength, CJMaxOut, CJMinOut, CJMaxAng, CJMinAng};
//  
//  MuscleOut R0Out;
//  R0Out = VMuscles(R0In, FR0, CR0);
//  
//  R0Out.CJPos = PreJointAng.at(CR0_m);
//  R0Out.CJVel = PreJointVel.at(CR0_m);
//  R0Out.CJNeu = m_reflex_muscle.at(CR0_m);
//  R0Out.FJPos = PreJointAng.at(FR0_m);
//  R0Out.FJVel = PreJointVel.at(FR0_m);
//  R0Out.FJNeu = m_reflex_muscle.at(FR0_m);
  
  

  

  
  



  //std::cout<<"K11 = "<<K11<<","<<"K22 = "<<K22<<"\n";
//  if (TESTSTIFF == 1)
//  {
//    if(SideForce.at(2) != 0)
//      {
//        if(SideForce.at(2) > 0 )
//         {
//           K2[0] = K2[0] + CTRSTIFFSETP;
//           K2[1] = K2[1] + CTRSTIFFSETP;
//           K2[2] = K2[2] + CTRSTIFFSETP;
//
//
//           K2[3] = K22;
//           K2[4] = K22;
//           K2[5] = K22;
//
//         }
//        else
//        {
//          K2[3] = K2[3] + CTRSTIFFSETP;
//          K2[4] = K2[4] + CTRSTIFFSETP;
//          K2[5] = K2[5] + CTRSTIFFSETP;
//
//          K2[0] = K22;
//          K2[1] = K22;
//          K2[2] = K22;
//
//        }
//
//      }
//
//  }





if (WHMUSCLEALLPHA == 0)
{
  


  
  MusclePara CR0Out, FR0Out;
  
  if (m_pre.at(TR0_m) < m_preold.at(TR0_m))// && (Raw_R_fs.at(1) > FSERR)
   {
//    m_Prereflex.at(FR0_m) = Scaling(m_Prereflex.at(FR0_m), -0.67, -1.0, 1.0, -0.9);
//    m_Prereflex.at(CR0_m) = Scaling(m_Prereflex.at(CR0_m), 1.0, 0.71, 1.0, -1.0);
//    m_reflex.at(FR0_m) = Scaling(m_reflex.at(FR0_m), -0.67, -1.0, 1.0, -0.9);
//    m_reflex.at(CR0_m) = Scaling(m_reflex.at(CR0_m), 1.0, 0.71, 1.0, -1.0);
    
    Exp_R_fs.at(0) = 1.0;

    Exp_R_fs.at(0) = FCProcessing(Exp_R_fs.at(0), Exp_R_fs_old.at(0));

     MusclePara ParaR11 = RungeKutta41(PreJointAng.at(FR0_m), PreJointVel.at(FR0_m), SampleTime , Raw_R_fs_old.at(0), 
         Raw_R_fs.at(0), m_preold.at(FR0_m), m_pre.at(FR0_m), 0, K1[0], D1[0], HipLength, 0, FMusclActFac);
     
     FR0Out.Pos = ParaR11.Pos;
     PreJointAng.at(FR0_m) = ParaR11.Pos;
     m_reflex_muscle.at(FR0_m) = Scaling(FR0Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
     FR0Out.Vel = ParaR11.Vel;
     PreJointVel.at(FR0_m) =  ParaR11.Vel;
     
     double DisVe = (HipLength + Radius) * sin(ParaR11.Pos);
//     MusclePara ParaR12 = RungeKutta42(PreJointAng.at(CR0_m), PreJointVel.at(CR0_m), SampleTime , Raw_R_fs_old.at(0), 
//         Raw_R_fs.at(0), m_Prereflex.at(CR0_m), m_reflex.at(CR0_m), DisVe, K2[0], D2[0], KneeLength, 0, CMusclActFac);
     
     MusclePara ParaR12 = RungeKutta42(PreJointAng.at(CR0_m), PreJointVel.at(CR0_m), SampleTime , Raw_R_fs_old.at(0), 
         Raw_R_fs.at(0), m_preold.at(CR0_m), m_pre.at(CR0_m), DisVe, K2[0], D2[0], KneeLength, 0, CMusclActFac);
     
     CR0Out.Pos = ParaR12.Pos;
     PreJointAng.at(CR0_m) = ParaR12.Pos;
     m_reflex_muscle.at(CR0_m) = Scaling(CR0Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
     CR0Out.Vel = ParaR12.Vel;
     PreJointVel.at(CR0_m) = ParaR12.Vel;

     Diff_R_fs.at(0) = Raw_R_fs.at(0) - Exp_R_fs.at(0);

     acc_Diff_R_fs.at(0) += (-Diff_R_fs.at(0));


     TouchR0 = TouchR0 + 1;


      AveDiff_R_fs.at(0) = (abs(Diff_R_fs.at(0)) + (TouchR0 - 1) * OldAveDiff_R_fs.at(0))/TouchR0;



      if (m_reflex_muscle.at(CR0_m) > MINCTROUT)
       {
         m_reflex_muscle.at(CR0_m) = MINCTROUT;
       }

      if (LEARNINGTYPE == 9)
      {
        K2[0] = TSSDualRateLearning(Raw_R_fs.at(0), K2[0], Exp_R_fs.at(0),dSlowR0,dFastR0,CTRBASELINE);
            Stiff_R.at(0) = K2[0];
            std::cout<<"The CTr stiffness of R0 is"<<K2[0]<<"\n";

            K1[0] = TSSDualRateLearning(Raw_R_fs.at(0), K1[0], Exp_R_fs.at(0),dFTiSlowR0,dFTiFastR0,FTIBASELINE);
                FTiStiff_R.at(0) = K1[0];
                std::cout<<"The FTi stiffness of R0 is"<<K1[0]<<"\n";

                Stiff_R_Fast.at(0) = dFastR0;
                Stiff_R_Slow.at(0) = dSlowR0;

                FTiStiff_R_Fast.at(0) = dFTiFastR0;
                FTiStiff_R_Slow.at(0) = dFTiSlowR0;



      }



     
   }
   else
   {
     Exp_R_fs.at(0) = 0.0;

     m_reflex_muscle.at(FR0_m) = dSWFTiFac * m_reflex.at(FR0_m) - dSWFTiOffset;
     PreJointAng.at(FR0_m) = Scaling(m_reflex.at(FR0_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
     PreJointVel.at(FR0_m) = 0;
     m_reflex_muscle.at(CR0_m) = m_reflex.at(CR0_m);
     PreJointAng.at(CR0_m) = Scaling(m_reflex.at(CR0_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
     PreJointVel.at(CR0_m) = 0;
     
     TouchR0 = 0;

     acc_Diff_R_fs.at(0) = 0;

//     if (AveDiff_R_fs.at(0) > 0)
//  {
//       dAveErrorR0 =  AveDiff_R_fs.at(0);
//
//       if ((dAveErrorR0 > FCERROR) and (K2[0] < MAXCTRSTIFF))
//       {
//         K2[0] = K2[0] + CTRSTIFFSETP * dAveErrorR0;
//         std::cout<<"The CTr stiffness of R0 is"<<K2[0]<<"\n";
//       }
//
//  }
          if (AveDiff_R_fs.at(0) > 0)
       {

            if ((LEARNINGTYPE == 6) || (LEARNINGTYPE == 8))
            {
              if(AveDiff_R_fs.at(0) > LTDRLFCERROR)
              {
                dLDRLR0 = dLDRLR0 + AveDiff_R_fs.at(0);
              }

            }

            if (LEARNINGTYPE == 0)
            {
              K2[0] =  LMSLearning(AveDiff_R_fs.at(0), CR0_m, K2[0]);
              Stiff_R.at(0) = K2[0];

            }

       }



     AveDiff_R_fs.at(0) = 0;

   }
  
  if (LEARNINGTYPE == 1)
  {
    K2[0] = RLSLearning(Raw_R_fs.at(0), K2[0], Exp_R_fs.at(0), dGainR0, dAutoCorR0);

    Stiff_R.at(0) = K2[0];
    std::cout<<"The CTr stiffness of R0 is"<<K2[0]<<"\n";
  }

  if (LEARNINGTYPE == 2)
  {
    K2[0] = DualRateLearning(Raw_R_fs.at(0), K2[0], Exp_R_fs.at(0));
        Stiff_R.at(0) = K2[0];
        std::cout<<"The CTr stiffness of R0 is"<<K2[0]<<"\n";
  }

  if (LEARNINGTYPE == 3)
  {
    K2[0] = TDualRateLearning(Raw_R_fs.at(0), K2[0], Exp_R_fs.at(0));
        Stiff_R.at(0) = K2[0];
        std::cout<<"The CTr stiffness of R0 is"<<K2[0]<<"\n";
  }

  if (LEARNINGTYPE == 6)
  {
    K2[0] = LTDualRateLearning(Raw_R_fs.at(0), K2[0], Exp_R_fs.at(0),dLDRLR0);
        Stiff_R.at(0) = K2[0];
        std::cout<<"The CTr stiffness of R0 is"<<K2[0]<<"\n";
  }

  if (LEARNINGTYPE == 7)
  {
    K2[0] = SDualRateLearning(Raw_R_fs.at(0), K2[0], Exp_R_fs.at(0),dSlowR0,dFastR0);
        Stiff_R.at(0) = K2[0];
        std::cout<<"The CTr stiffness of R0 is"<<K2[0]<<"\n";
  }

  if (LEARNINGTYPE == 8)
  {
    K2[0] = ParaSDualRateLearning(Raw_R_fs.at(0), K2[0], Exp_R_fs.at(0),dSlowR0,dFastR0,dLDRLR0);
        Stiff_R.at(0) = K2[0];
        std::cout<<"The CTr stiffness of R0 is"<<K2[0]<<"\n";
  }

  if (LEARNINGTYPE == 4)
  {
    K2[0] = FastLearning(Raw_R_fs.at(0), K2[0], Exp_R_fs.at(0));
        Stiff_R.at(0) = K2[0];
        std::cout<<"The CTr stiffness of R0 is"<<K2[0]<<"\n";
  }

  if (LEARNINGTYPE == 5)
  {
    K2[0] = SlowLearning(Raw_R_fs.at(0), K2[0], Exp_R_fs.at(0));
        Stiff_R.at(0) = K2[0];
        std::cout<<"The CTr stiffness of R0 is"<<K2[0]<<"\n";
  }



  MusclePara CR1Out, FR1Out;
   
   if (m_pre.at(TR1_m) < m_preold.at(TR1_m))// && (Raw_R_fs.at(1) > FSERR)
    {
//     m_Prereflex.at(FR1_m) = Scaling(m_Prereflex.at(FR1_m), -0.67, -1.0, 1.0, -0.9);
//     m_reflex.at(FR1_m) = Scaling(m_reflex.at(FR1_m), -0.67, -1.0, 1.0, -0.9);
//     m_Prereflex.at(CR1_m) = Scaling(m_Prereflex.at(CR1_m), 1.0, 0.71, 1.0, -1.0);
//     m_reflex.at(CR1_m) = Scaling(m_reflex.at(CR1_m), 1.0, 0.71, 1.0, -1.0);
      
     Exp_R_fs.at(1) = 1.0;
     Exp_R_fs.at(1) = FCProcessing(Exp_R_fs.at(1), Exp_R_fs_old.at(1));

      MusclePara ParaR21 = RungeKutta41(PreJointAng.at(FR1_m), PreJointVel.at(FR1_m), SampleTime , Raw_R_fs_old.at(1), 
          Raw_R_fs.at(1), m_preold.at(FR1_m), m_pre.at(FR1_m), 0, K1[1], D1[1], HipLength, 0, FMusclActFac);
      
      FR1Out.Pos = ParaR21.Pos;
      PreJointAng.at(FR1_m) = ParaR21.Pos;
      m_reflex_muscle.at(FR1_m) = Scaling(FR1Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
      FR1Out.Vel = ParaR21.Vel;
      PreJointVel.at(FR1_m) = ParaR21.Vel;
                  
      double DisVe1 = (HipLength + Radius) * sin(ParaR21.Pos);
//      MusclePara ParaR22 = RungeKutta42(PreJointAng.at(CR1_m), PreJointVel.at(CR1_m), SampleTime , Raw_R_fs_old.at(1), 
//          Raw_R_fs.at(1), m_Prereflex.at(CR1_m), m_reflex.at(CR1_m), DisVe1, K2[1], D2[1], KneeLength, 0, CMusclActFac);
      
      MusclePara ParaR22 = RungeKutta42(PreJointAng.at(CR1_m), PreJointVel.at(CR1_m), SampleTime , Raw_R_fs_old.at(1), 
          Raw_R_fs.at(1),  m_preold.at(CR1_m), m_pre.at(CR1_m), DisVe1, K2[1], D2[1], KneeLength, 0, CMusclActFac);
      
      CR1Out.Pos = ParaR22.Pos;
      PreJointAng.at(CR1_m) = ParaR22.Pos;
      m_reflex_muscle.at(CR1_m) = Scaling(CR1Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
      CR1Out.Vel = ParaR22.Vel;
      PreJointVel.at(CR1_m) = ParaR22.Vel;


      Diff_R_fs.at(1) = Raw_R_fs.at(1) - Exp_R_fs.at(1);

      TouchR1 = TouchR1 + 1;


       AveDiff_R_fs.at(1) = (abs(Diff_R_fs.at(1)) + (TouchR1 - 1) * OldAveDiff_R_fs.at(1))/TouchR1;

      if (m_reflex_muscle.at(CR1_m) > MINCTROUT)
          {
            m_reflex_muscle.at(CR1_m) = MINCTROUT;
          }

      if(LEARNINGTYPE == 9)
       {
         K2[1] = TSSDualRateLearning(Raw_R_fs.at(1), K2[1], Exp_R_fs.at(1),dSlowR1,dFastR1,CTRBASELINE);
         Stiff_R.at(1) = K2[1];
         std::cout<<"The CTr stiffness of R1 is"<<K2[1]<<"\n";


         K1[1] = TSSDualRateLearning(Raw_R_fs.at(1), K1[1], Exp_R_fs.at(1),dFTiSlowR1,dFTiFastR1,FTIBASELINE);
             FTiStiff_R.at(1) = K1[1];
             std::cout<<"The FTi stiffness of R0 is"<<K1[1]<<"\n";

             Stiff_R_Fast.at(1) = dFastR1;
             Stiff_R_Slow.at(1) = dSlowR1;

             FTiStiff_R_Fast.at(1) = dFTiFastR1;
             FTiStiff_R_Slow.at(1) = dFTiSlowR1;

       }

      
    }
    else
    {
      Exp_R_fs.at(1) = 0.0;

      m_reflex_muscle.at(FR1_m) = dSWFTiFac * m_reflex.at(FR1_m) - dSWFTiOffset;
      PreJointAng.at(FR1_m) = Scaling(m_reflex.at(FR1_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
      PreJointVel.at(FR1_m) = 0;
      m_reflex_muscle.at(CR1_m) = m_reflex.at(CR1_m);
      PreJointAng.at(CR1_m) = Scaling(m_reflex.at(CR1_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
      PreJointVel.at(CR1_m) = 0;
      
      TouchR1 = 0;
//
//        if (AveDiff_R_fs.at(1) > 0)
//     {
//          dAveErrorR1 =  AveDiff_R_fs.at(1);
//
//          if ((dAveErrorR1 > FCERROR) and (K2[1] < MAXCTRSTIFF))
//          {
//            K2[1] = K2[1] + CTRSTIFFSETP * dAveErrorR1;
//            std::cout<<"The CTr stiffness of R1 is"<<K2[1]<<"\n";
//          }
//
//     }

              if (AveDiff_R_fs.at(1) > 0)
           {

                if ((LEARNINGTYPE == 6) || (LEARNINGTYPE == 8))
                {
                  if (AveDiff_R_fs.at(1) > LTDRLFCERROR)
                  {
                    dLDRLR1 = dLDRLR1 + AveDiff_R_fs.at(1);

                  }

                }

                if (LEARNINGTYPE == 0)
                {
                  K2[1] =  LMSLearning(AveDiff_R_fs.at(1), CR1_m, K2[1]);
                  Stiff_R.at(1) = K2[1];

                }

           }


        AveDiff_R_fs.at(1) = 0;

    }
   
   if (LEARNINGTYPE == 1)
   {
     K2[1] = RLSLearning(Raw_R_fs.at(1), K2[1], Exp_R_fs.at(1), dGainR1, dAutoCorR1);
     Stiff_R.at(1) = K2[1];
     std::cout<<"The CTr stiffness of R1 is"<<K2[1]<<"\n";
   }

   if(LEARNINGTYPE == 2)
   {
     K2[1] = DualRateLearning(Raw_R_fs.at(1), K2[1], Exp_R_fs.at(1));
     Stiff_R.at(1) = K2[1];
     std::cout<<"The CTr stiffness of R1 is"<<K2[1]<<"\n";
   }

   if(LEARNINGTYPE == 3)
   {
     K2[1] = TDualRateLearning(Raw_R_fs.at(1), K2[1], Exp_R_fs.at(1));
     Stiff_R.at(1) = K2[1];
     std::cout<<"The CTr stiffness of R1 is"<<K2[1]<<"\n";
   }

   if(LEARNINGTYPE == 6)
    {
      K2[1] = LTDualRateLearning(Raw_R_fs.at(1), K2[1], Exp_R_fs.at(1),dLDRLR1);
      Stiff_R.at(1) = K2[1];
      std::cout<<"The CTr stiffness of R1 is"<<K2[1]<<"\n";
    }

   if(LEARNINGTYPE == 7)
    {
      K2[1] = SDualRateLearning(Raw_R_fs.at(1), K2[1], Exp_R_fs.at(1),dSlowR1,dFastR1);
      Stiff_R.at(1) = K2[1];
      std::cout<<"The CTr stiffness of R1 is"<<K2[1]<<"\n";
    }

   if(LEARNINGTYPE == 8)
    {
      K2[1] = ParaSDualRateLearning(Raw_R_fs.at(1), K2[1], Exp_R_fs.at(1),dSlowR1,dFastR1,dLDRLR1);
      Stiff_R.at(1) = K2[1];
      std::cout<<"The CTr stiffness of R1 is"<<K2[1]<<"\n";
    }


   if(LEARNINGTYPE == 4)
   {
     K2[1] = FastLearning(Raw_R_fs.at(1), K2[1], Exp_R_fs.at(1));
     Stiff_R.at(1) = K2[1];
     std::cout<<"The CTr stiffness of R1 is"<<K2[1]<<"\n";
   }

   if(LEARNINGTYPE == 5)
   {
     K2[1] = SlowLearning(Raw_R_fs.at(1), K2[1], Exp_R_fs.at(1));
     Stiff_R.at(1) = K2[1];
     std::cout<<"The CTr stiffness of R1 is"<<K2[1]<<"\n";
   }

   MusclePara CR2Out, FR2Out;
    
    if (m_pre.at(TR2_m) < m_preold.at(TR2_m))// && (Raw_R_fs.at(1) > FSERR)
     {
      
//      m_Prereflex.at(FR2_m) = Scaling(m_Prereflex.at(FR2_m), -0.67, -1.0, 1.0, -0.9);
//      m_reflex.at(FR2_m) = Scaling(m_reflex.at(FR2_m), -0.67, -1.0, 1.0, -0.9);
//      m_Prereflex.at(CR2_m) = Scaling(m_Prereflex.at(CR2_m), 1.0, 0.71, 1.0, -1.0);
//      m_reflex.at(CR2_m) = Scaling(m_reflex.at(CR2_m), 1.0, 0.71, 1.0, -1.0);
      Exp_R_fs.at(2) = 1.0;
      Exp_R_fs.at(2) = FCProcessing(Exp_R_fs.at(2), Exp_R_fs_old.at(2));

       MusclePara ParaR31 = RungeKutta41(PreJointAng.at(FR2_m), PreJointVel.at(FR2_m), SampleTime , Raw_R_fs_old.at(2), 
           Raw_R_fs.at(2), m_preold.at(FR2_m), m_pre.at(FR2_m), 0, K1[2], D1[2], HipLength, 0, FMusclActFac);
       
       FR2Out.Pos = ParaR31.Pos;
       PreJointAng.at(FR2_m) = ParaR31.Pos;
       m_reflex_muscle.at(FR2_m) = Scaling(FR2Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
       FR2Out.Vel = ParaR31.Vel;
       PreJointVel.at(FR2_m) = ParaR31.Vel;
                   
       double DisVe2 = (HipLength + Radius) * sin(ParaR31.Pos);
//       MusclePara ParaR32 = RungeKutta42(PreJointAng.at(CR2_m), PreJointVel.at(CR2_m), SampleTime , Raw_R_fs_old.at(2), 
//           Raw_R_fs.at(2), m_Prereflex.at(CR2_m), m_reflex.at(CR2_m), DisVe2, K2[2], D2[2], KneeLength, 0, CMusclActFac);
       
       MusclePara ParaR32 = RungeKutta42(PreJointAng.at(CR2_m), PreJointVel.at(CR2_m), SampleTime , Raw_R_fs_old.at(2), 
                Raw_R_fs.at(2), m_preold.at(CR2_m), m_pre.at(CR2_m), DisVe2, K2[2], D2[2], KneeLength, 0, CMusclActFac);
            
       
       CR2Out.Pos = ParaR32.Pos;
       PreJointAng.at(CR2_m) = ParaR32.Pos;
       m_reflex_muscle.at(CR2_m) = Scaling(CR2Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
       CR2Out.Vel = ParaR32.Vel;
       PreJointVel.at(CR2_m) = ParaR32.Vel;

       Diff_R_fs.at(2) = Raw_R_fs.at(2) - Exp_R_fs.at(2);

       TouchR2 = TouchR2 + 1;


        AveDiff_R_fs.at(2) = (abs(Diff_R_fs.at(2)) + (TouchR2 - 1) * OldAveDiff_R_fs.at(2))/TouchR2;

       if (m_reflex_muscle.at(CR2_m) > MINCTROUT)
           {
             m_reflex_muscle.at(CR2_m) = MINCTROUT;
           }


       if (LEARNINGTYPE == 9)
       {
         K2[2] = TSSDualRateLearning(Raw_R_fs.at(2), K2[2], Exp_R_fs.at(2),dSlowR2,dFastR2,CTRBASELINE);
         Stiff_R.at(2) = K2[2];
         std::cout<<"The CTr stiffness of R2 is"<<K2[2]<<"\n";

         K1[2] = TSSDualRateLearning(Raw_R_fs.at(2), K1[2], Exp_R_fs.at(2),dFTiSlowR2,dFTiFastR2,FTIBASELINE);
             FTiStiff_R.at(2) = K1[2];
             std::cout<<"The FTi stiffness of R0 is"<<K1[2]<<"\n";


             Stiff_R_Fast.at(2) = dFastR2;
             Stiff_R_Slow.at(2) = dSlowR2;

             FTiStiff_R_Fast.at(2) = dFTiFastR2;
             FTiStiff_R_Slow.at(2) = dFTiSlowR2;

       }

       
     }
     else
     {
       Exp_R_fs.at(2) = 0.0;

       m_reflex_muscle.at(FR2_m) = dSWFTiFac * m_reflex.at(FR2_m) - dSWFTiOffset;
       PreJointAng.at(FR2_m) = Scaling(m_reflex.at(FR2_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
       PreJointVel.at(FR2_m) = 0;
       m_reflex_muscle.at(CR2_m) = m_reflex.at(CR2_m);
       PreJointAng.at(CR2_m) = Scaling(m_reflex.at(CR2_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
       PreJointVel.at(CR2_m) = 0;
       
       TouchR2 = 0;

//         if (AveDiff_R_fs.at(2) > 0)
//      {
//           dAveErrorR2 =  AveDiff_R_fs.at(2);
//
//           if ((dAveErrorR2 > FCERROR) and (K2[2] < MAXCTRSTIFF))
//           {
//             K2[2] = K2[2] + CTRSTIFFSETP * dAveErrorR2;
//             std::cout<<"The CTr stiffness of R2 is"<<K2[2]<<"\n";
//           }
//
//      }

                if (AveDiff_R_fs.at(2) > 0)
             {
                  if ((LEARNINGTYPE == 6) || (LEARNINGTYPE == 8))
                  {
                    if (AveDiff_R_fs.at(2) > LTDRLFCERROR)
                    {
                      dLDRLR2 = dLDRLR2 + AveDiff_R_fs.at(2);

                    }

                  }

                  if (LEARNINGTYPE == 0)
                  {
                    K2[2] =  LMSLearning(AveDiff_R_fs.at(2), CR2_m, K2[2]);
                    Stiff_R.at(2) = K2[2];

                  }

             }

         AveDiff_R_fs.at(2) = 0;

     }
    
    if (LEARNINGTYPE == 1)
    {
      K2[2] = RLSLearning(Raw_R_fs.at(2), K2[2], Exp_R_fs.at(2), dGainR2, dAutoCorR2);
      Stiff_R.at(2) = K2[2];
      std::cout<<"The CTr stiffness of R2 is"<<K2[2]<<"\n";
    }

    if (LEARNINGTYPE == 2)
    {
      K2[2] = DualRateLearning(Raw_R_fs.at(2), K2[2], Exp_R_fs.at(2));
      Stiff_R.at(2) = K2[2];
      std::cout<<"The CTr stiffness of R2 is"<<K2[2]<<"\n";
    }

    if (LEARNINGTYPE == 3)
    {
      K2[2] = TDualRateLearning(Raw_R_fs.at(2), K2[2], Exp_R_fs.at(2));
      Stiff_R.at(2) = K2[2];
      std::cout<<"The CTr stiffness of R2 is"<<K2[2]<<"\n";
    }

    if (LEARNINGTYPE == 6)
    {
      K2[2] = LTDualRateLearning(Raw_R_fs.at(2), K2[2], Exp_R_fs.at(2),dLDRLR2);
      Stiff_R.at(2) = K2[2];
      std::cout<<"The CTr stiffness of R2 is"<<K2[2]<<"\n";
    }

    if (LEARNINGTYPE == 7)
    {
      K2[2] = SDualRateLearning(Raw_R_fs.at(2), K2[2], Exp_R_fs.at(2),dSlowR2,dFastR2);
      Stiff_R.at(2) = K2[2];
      std::cout<<"The CTr stiffness of R2 is"<<K2[2]<<"\n";
    }

    if (LEARNINGTYPE == 8)
    {
      K2[2] = ParaSDualRateLearning(Raw_R_fs.at(2), K2[2], Exp_R_fs.at(2),dSlowR2,dFastR2,dLDRLR2);
      Stiff_R.at(2) = K2[2];
      std::cout<<"The CTr stiffness of R2 is"<<K2[2]<<"\n";
    }

    if (LEARNINGTYPE == 4)
    {
      K2[2] = FastLearning(Raw_R_fs.at(2), K2[2], Exp_R_fs.at(2));
      Stiff_R.at(2) = K2[2];
      std::cout<<"The CTr stiffness of R2 is"<<K2[2]<<"\n";
    }

    if (LEARNINGTYPE == 5)
    {
      K2[2] = SlowLearning(Raw_R_fs.at(2), K2[2], Exp_R_fs.at(2));
      Stiff_R.at(2) = K2[2];
      std::cout<<"The CTr stiffness of R2 is"<<K2[2]<<"\n";
    }




    
    MusclePara CL0Out, FL0Out;
      
     if (m_pre.at(TL0_m) < m_preold.at(TL0_m))// && (Raw_R_fs.at(1) > FSERR)
      {
       
//       m_Prereflex.at(FL0_m) = Scaling(m_Prereflex.at(FL0_m), -0.67, -1.0, 1.0, -0.9);
//       m_reflex.at(FL0_m) = Scaling(m_reflex.at(FL0_m), -0.67, -1.0, 1.0, -0.9);
//       m_Prereflex.at(CL0_m) = Scaling(m_Prereflex.at(CL0_m), 1.0, 0.71, 1.0, -1.0);
//       m_reflex.at(CL0_m) = Scaling(m_reflex.at(CL0_m), 1.0, 0.71, 1.0, -1.0);
//
       Exp_L_fs.at(0) = 1.0;
       Exp_L_fs.at(0) = FCProcessing(Exp_L_fs.at(0), Exp_L_fs_old.at(0));

         MusclePara ParaR41 = RungeKutta41(PreJointAng.at(FL0_m), PreJointVel.at(FL0_m), SampleTime , Raw_L_fs_old.at(0), 
             Raw_L_fs.at(0), m_preold.at(FL0_m), m_pre.at(FL0_m), 0, K1[3], D1[3], HipLength, 0, FMusclActFac);
         
         FL0Out.Pos = ParaR41.Pos;
         PreJointAng.at(FL0_m) = ParaR41.Pos;
         m_reflex_muscle.at(FL0_m) = Scaling(FL0Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
         FL0Out.Vel = ParaR41.Vel;
         PreJointVel.at(FL0_m) = ParaR41.Vel;
                     
         double DisVe3 = (HipLength + Radius) * sin(ParaR41.Pos);
//         MusclePara ParaR42 = RungeKutta42(PreJointAng.at(CL0_m), PreJointVel.at(CL0_m), SampleTime , Raw_L_fs_old.at(0), 
//             Raw_L_fs.at(0), m_Prereflex.at(CL0_m), m_reflex.at(CL0_m), DisVe3, K2[3], D2[3], KneeLength, 0, CMusclActFac);
         
         MusclePara ParaR42 = RungeKutta42(PreJointAng.at(CL0_m), PreJointVel.at(CL0_m), SampleTime , Raw_L_fs_old.at(0), 
             Raw_L_fs.at(0), m_preold.at(CL0_m), m_pre.at(CL0_m), DisVe3, K2[3], D2[3], KneeLength, 0, CMusclActFac);
         
         CL0Out.Pos = ParaR42.Pos;
         PreJointAng.at(CL0_m) = ParaR42.Pos;
         m_reflex_muscle.at(CL0_m) = Scaling(CL0Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
         CL0Out.Vel = ParaR42.Vel;
         PreJointVel.at(CL0_m) = ParaR42.Vel;



         Diff_L_fs.at(0) = Raw_L_fs.at(0) - Exp_L_fs.at(0);

         TouchL0 = TouchL0 + 1;


          AveDiff_L_fs.at(0) = (abs(Diff_L_fs.at(0)) + (TouchL0 - 1) * OldAveDiff_L_fs.at(0))/TouchL0;



         if (m_reflex_muscle.at(CL0_m) > MINCTROUT)
                 {
                   m_reflex_muscle.at(CL0_m) = MINCTROUT;
                 }

         if (LEARNINGTYPE == 9)
         {
           K2[3] = TSSDualRateLearning(Raw_L_fs.at(0), K2[3], Exp_L_fs.at(0),dSlowL0,dFastL0,CTRBASELINE);
           Stiff_L.at(0) = K2[3];
           std::cout<<"The CTr stiffness of L0 is"<<K2[3]<<"\n";

           K1[3] = TSSDualRateLearning(Raw_L_fs.at(0), K1[3], Exp_L_fs.at(0),dFTiSlowL0,dFTiFastL0,FTIBASELINE);
               FTiStiff_L.at(0) = K1[3];
               std::cout<<"The FTi stiffness of R0 is"<<K1[3]<<"\n";


               Stiff_L_Fast.at(0) = dFastL0;
               Stiff_L_Slow.at(0) = dSlowL0;

               FTiStiff_L_Fast.at(0) = dFTiFastL0;
               FTiStiff_L_Slow.at(0) = dFTiSlowL0;
         }


         
       }
       else
       {
         
         Exp_L_fs.at(0) = 0.0;
         m_reflex_muscle.at(FL0_m) = dSWFTiFac * m_reflex.at(FL0_m) - dSWFTiOffset;
         PreJointAng.at(FL0_m) = Scaling(m_reflex.at(FL0_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
         PreJointVel.at(FL0_m) = 0;
         m_reflex_muscle.at(CL0_m) = m_reflex.at(CL0_m);
         PreJointAng.at(CL0_m) = Scaling(m_reflex.at(CL0_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
         PreJointVel.at(CL0_m) = 0;
         
         TouchL0 = 0;

//           if (AveDiff_L_fs.at(0) > 0)
//        {
//             dAveErrorL0 =  AveDiff_L_fs.at(0);
//
//             if ((dAveErrorL0 > FCERROR) and (K2[3] < MAXCTRSTIFF))
//             {
//               K2[3] = K2[3] + CTRSTIFFSETP * dAveErrorL0;
//               std::cout<<"The CTr stiffness of L0 is"<<K2[3]<<"\n";
//             }
//
//        }

                    if (AveDiff_L_fs.at(0) > 0)
                 {
                      if ((LEARNINGTYPE == 6) || (LEARNINGTYPE == 8))
                      {
                        if(AveDiff_L_fs.at(0) > LTDRLFCERROR)
                        {
                          dLDRLL0 = dLDRLL0 + AveDiff_L_fs.at(0);
                        }

                      }

                      if (LEARNINGTYPE == 0)
                      {

                        K2[3] =  LMSLearning(AveDiff_L_fs.at(0), CL0_m, K2[3]);
                        Stiff_L.at(0) = K2[3];

                      }

                 }

           AveDiff_L_fs.at(0) = 0;

       }
     
     if (LEARNINGTYPE == 1)
     {
       K2[3] = RLSLearning(Raw_L_fs.at(0), K2[3], Exp_L_fs.at(0), dGainL0, dAutoCorL0);
       Stiff_L.at(0) = K2[3];
       std::cout<<"The CTr stiffness of L0 is"<<K2[3]<<"\n";
     }

     if (LEARNINGTYPE == 2)
     {
       K2[3] = DualRateLearning(Raw_L_fs.at(0), K2[3], Exp_L_fs.at(0));
       Stiff_L.at(0) = K2[3];
       std::cout<<"The CTr stiffness of L0 is"<<K2[3]<<"\n";
     }

     if (LEARNINGTYPE == 3)
     {
       K2[3] = TDualRateLearning(Raw_L_fs.at(0), K2[3], Exp_L_fs.at(0));
       Stiff_L.at(0) = K2[3];
       std::cout<<"The CTr stiffness of L0 is"<<K2[3]<<"\n";
     }

     if (LEARNINGTYPE == 6)
     {
       K2[3] = LTDualRateLearning(Raw_L_fs.at(0), K2[3], Exp_L_fs.at(0),dLDRLL0);
       Stiff_L.at(0) = K2[3];
       std::cout<<"The CTr stiffness of L0 is"<<K2[3]<<"\n";
     }

     if (LEARNINGTYPE == 7)
     {
       K2[3] = SDualRateLearning(Raw_L_fs.at(0), K2[3], Exp_L_fs.at(0),dSlowL0,dFastL0);
       Stiff_L.at(0) = K2[3];
       std::cout<<"The CTr stiffness of L0 is"<<K2[3]<<"\n";
     }

     if (LEARNINGTYPE == 8)
     {
       K2[3] = ParaSDualRateLearning(Raw_L_fs.at(0), K2[3], Exp_L_fs.at(0),dSlowL0,dFastL0,dLDRLL0);
       Stiff_L.at(0) = K2[3];
       std::cout<<"The CTr stiffness of L0 is"<<K2[3]<<"\n";
     }

     if (LEARNINGTYPE == 4)
     {
       K2[3] = FastLearning(Raw_L_fs.at(0), K2[3], Exp_L_fs.at(0));
       Stiff_L.at(0) = K2[3];
       std::cout<<"The CTr stiffness of L0 is"<<K2[3]<<"\n";
     }

     if (LEARNINGTYPE == 5)
     {
       K2[3] = SlowLearning(Raw_L_fs.at(0), K2[3], Exp_L_fs.at(0));
       Stiff_L.at(0) = K2[3];
       std::cout<<"The CTr stiffness of L0 is"<<K2[3]<<"\n";
     }

     MusclePara CL1Out, FL1Out;
       
      if (m_pre.at(TL1_m) < m_preold.at(TL1_m))// && (Raw_R_fs.at(1) > FSERR)
       {
//        m_Prereflex.at(FL1_m) = Scaling(m_Prereflex.at(FL1_m), -0.67, -1.0, 1.0, -0.9);
//        m_reflex.at(FL1_m) = Scaling(m_reflex.at(FL1_m), -0.67, -1.0, 1.0, -0.9);
//        m_Prereflex.at(CL1_m) = Scaling(m_Prereflex.at(CL1_m), 1.0, 0.71, 1.0, -1.0);
//        m_reflex.at(CL1_m) = Scaling(m_reflex.at(CL1_m), 1.0, 0.71, 1.0, -1.0);
          

          Exp_L_fs.at(1) = 1.0;
          Exp_L_fs.at(1) = FCProcessing(Exp_L_fs.at(1), Exp_L_fs_old.at(1));

          MusclePara ParaR51 = RungeKutta41(PreJointAng.at(FL1_m), PreJointVel.at(FL1_m), SampleTime , Raw_L_fs_old.at(1), 
              Raw_L_fs.at(1), m_preold.at(FL1_m), m_pre.at(FL1_m), 0, K1[4], D1[4], HipLength, 0, FMusclActFac);
          
          FL1Out.Pos = ParaR51.Pos;
          PreJointAng.at(FL1_m) = ParaR51.Pos;
          m_reflex_muscle.at(FL1_m) = Scaling(FL1Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
          FL1Out.Vel = ParaR51.Vel;
          PreJointVel.at(FL1_m) = ParaR51.Vel;
                      
          double DisVe4 = (HipLength + Radius) * sin(ParaR51.Pos);
//          MusclePara ParaR52 = RungeKutta42(PreJointAng.at(CL1_m), PreJointVel.at(CL1_m), SampleTime , Raw_L_fs_old.at(1), 
//              Raw_L_fs.at(1), m_Prereflex.at(CL1_m), m_reflex.at(CL1_m), DisVe4, K2[4], D2[4], KneeLength, 0, CMusclActFac);
          
          MusclePara ParaR52 = RungeKutta42(PreJointAng.at(CL1_m), PreJointVel.at(CL1_m), SampleTime , Raw_L_fs_old.at(1), 
              Raw_L_fs.at(1), m_preold.at(CL1_m), m_pre.at(CL1_m), DisVe4, K2[4], D2[4], KneeLength, 0, CMusclActFac);
          
          CL1Out.Pos = ParaR52.Pos;
          PreJointAng.at(CL1_m) = ParaR52.Pos;
          m_reflex_muscle.at(CL1_m) = Scaling(CL1Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
          CL1Out.Vel = ParaR52.Vel;
          PreJointVel.at(CL1_m) = ParaR52.Vel;


          Diff_L_fs.at(1) = Raw_L_fs.at(1) - Exp_L_fs.at(1);

          TouchL1 = TouchL1 + 1;


           AveDiff_L_fs.at(1) = (abs(Diff_L_fs.at(1)) + (TouchL1 - 1) * OldAveDiff_L_fs.at(1))/TouchL1;

          if (m_reflex_muscle.at(CL1_m) > MINCTROUT)
                  {
                    m_reflex_muscle.at(CL1_m) = MINCTROUT;
                  }

          if (LEARNINGTYPE == 9)
          {
            K2[4] = TSSDualRateLearning(Raw_L_fs.at(1), K2[4], Exp_L_fs.at(1),dSlowL1,dFastL1,CTRBASELINE);
            Stiff_L.at(1) = K2[4];
            std::cout<<"The CTr stiffness of L1 is"<<K2[4]<<"\n";

            K1[4] = TSSDualRateLearning(Raw_L_fs.at(1), K1[4], Exp_L_fs.at(1),dFTiSlowL1,dFTiFastL1,FTIBASELINE);
                FTiStiff_L.at(1) = K1[4];
                std::cout<<"The FTi stiffness of R0 is"<<K1[4]<<"\n";

                Stiff_L_Fast.at(1) = dFastL1;
                Stiff_L_Slow.at(1) = dSlowL1;

                FTiStiff_L_Fast.at(1) = dFTiFastL1;
                FTiStiff_L_Slow.at(1) = dFTiSlowL1;
          }
  
          
        }
        else
        {
          
          Exp_L_fs.at(1) = 0.0;
          m_reflex_muscle.at(FL1_m) = dSWFTiFac * m_reflex.at(FL1_m) - dSWFTiOffset;
          PreJointAng.at(FL1_m) = Scaling(m_reflex.at(FL1_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
          PreJointVel.at(FL1_m) = 0;
          m_reflex_muscle.at(CL1_m) = m_reflex.at(CL1_m);
          PreJointAng.at(CL1_m) = Scaling(m_reflex.at(CL1_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
          PreJointVel.at(CL1_m) = 0;
          
          TouchL1 = 0;

//            if (AveDiff_L_fs.at(1) > 0)
//         {
//              dAveErrorL1 =  AveDiff_L_fs.at(1);
//
//              if ((dAveErrorL1 > FCERROR) and (K2[4] < MAXCTRSTIFF))
//              {
//                K2[4] = K2[4] + CTRSTIFFSETP * dAveErrorL1;
//                std::cout<<"The CTr stiffness of L1 is"<<K2[4]<<"\n";
//              }
//
//         }


                      if (AveDiff_L_fs.at(1) > 0)
                   {

                        if ((LEARNINGTYPE == 6) || (LEARNINGTYPE == 8))
                         {
                           if(AveDiff_L_fs.at(1) > LTDRLFCERROR)
                            {
                              dLDRLL1 = dLDRLL1 + AveDiff_L_fs.at(1);
                            }

                         }
                        if (LEARNINGTYPE == 0)
                        {
                          K2[4] =  LMSLearning(AveDiff_L_fs.at(1), CL1_m, K2[4]);
                          Stiff_L.at(1) = K2[4];

                        }

                   }


            AveDiff_L_fs.at(1) = 0;

        }
      
      if (LEARNINGTYPE == 1)
      {
        K2[4] = RLSLearning(Raw_L_fs.at(1), K2[4], Exp_L_fs.at(1), dGainL1, dAutoCorL1);
        Stiff_L.at(1) = K2[4];
        std::cout<<"The CTr stiffness of L1 is"<<K2[4]<<"\n";
      }

      if (LEARNINGTYPE == 2)
      {
        K2[4] = DualRateLearning(Raw_L_fs.at(1), K2[4], Exp_L_fs.at(1));
        Stiff_L.at(1) = K2[4];
        std::cout<<"The CTr stiffness of L1 is"<<K2[4]<<"\n";
      }

      if (LEARNINGTYPE == 3)
      {
        K2[4] = TDualRateLearning(Raw_L_fs.at(1), K2[4], Exp_L_fs.at(1));
        Stiff_L.at(1) = K2[4];
        std::cout<<"The CTr stiffness of L1 is"<<K2[4]<<"\n";
      }

      if (LEARNINGTYPE == 6)
      {
        K2[4] = LTDualRateLearning(Raw_L_fs.at(1), K2[4], Exp_L_fs.at(1),dLDRLL1);
        Stiff_L.at(1) = K2[4];
        std::cout<<"The CTr stiffness of L1 is"<<K2[4]<<"\n";
      }

      if (LEARNINGTYPE == 7)
      {
        K2[4] = SDualRateLearning(Raw_L_fs.at(1), K2[4], Exp_L_fs.at(1),dSlowL1,dFastL1);
        Stiff_L.at(1) = K2[4];
        std::cout<<"The CTr stiffness of L1 is"<<K2[4]<<"\n";
      }

      if (LEARNINGTYPE == 8)
      {
        K2[4] = ParaSDualRateLearning(Raw_L_fs.at(1), K2[4], Exp_L_fs.at(1),dSlowL1,dFastL1,dLDRLL1);
        Stiff_L.at(1) = K2[4];
        std::cout<<"The CTr stiffness of L1 is"<<K2[4]<<"\n";
      }


      if (LEARNINGTYPE == 4)
      {
        K2[4] = FastLearning(Raw_L_fs.at(1), K2[4], Exp_L_fs.at(1));
        Stiff_L.at(1) = K2[4];
        std::cout<<"The CTr stiffness of L1 is"<<K2[4]<<"\n";
      }

      if (LEARNINGTYPE == 5)
      {
        K2[4] = SlowLearning(Raw_L_fs.at(1), K2[4], Exp_L_fs.at(1));
        Stiff_L.at(1) = K2[4];
        std::cout<<"The CTr stiffness of L1 is"<<K2[4]<<"\n";
      }

      MusclePara CL2Out, FL2Out;
          
         if (m_pre.at(TL2_m) < m_preold.at(TL2_m))// && (Raw_R_fs.at(1) > FSERR)
          {
           
//           m_Prereflex.at(CL2_m) = Scaling(m_Prereflex.at(CL2_m), 1.0, 0.71, 1.0, -1.0);
//           m_Prereflex.at(FL2_m) = Scaling(m_Prereflex.at(FL2_m), -0.67, -1.0, 1.0, -0.9);
//           m_reflex.at(FL2_m) = Scaling(m_reflex.at(FL2_m), -0.67, -1.0, 1.0, -0.9);
//           m_reflex.at(CL2_m) = Scaling(m_reflex.at(CL2_m), 1.0, 0.71, 1.0, -1.0);
           Exp_L_fs.at(2) = 1.0;
           Exp_L_fs.at(2) = FCProcessing(Exp_L_fs.at(2), Exp_L_fs_old.at(2));

             MusclePara ParaR61 = RungeKutta41(PreJointAng.at(FL2_m), PreJointVel.at(FL2_m), SampleTime , Raw_L_fs_old.at(2), 
                 Raw_L_fs.at(2), m_preold.at(FL2_m), m_pre.at(FL2_m), 0, K1[5], D1[5], HipLength, 0, FMusclActFac);
             
             FL2Out.Pos = ParaR61.Pos;
             PreJointAng.at(FL2_m) = ParaR61.Pos;
             m_reflex_muscle.at(FL2_m) = Scaling(FL2Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
             FL2Out.Vel = ParaR61.Vel;
             PreJointVel.at(FL2_m) = ParaR61.Vel;
                         
             double DisVe5 = (HipLength + Radius) * sin(ParaR61.Pos);
//             MusclePara ParaR62 = RungeKutta42(PreJointAng.at(CL2_m), PreJointVel.at(CL2_m), SampleTime , Raw_L_fs_old.at(2), 
//                 Raw_L_fs.at(2), m_Prereflex.at(CL2_m), m_reflex.at(CL2_m), DisVe5, K2[5], D2[5], KneeLength, 0, CMusclActFac);
             
             MusclePara ParaR62 = RungeKutta42(PreJointAng.at(CL2_m), PreJointVel.at(CL2_m), SampleTime , Raw_L_fs_old.at(2), 
                              Raw_L_fs.at(2), m_preold.at(CL2_m), m_pre.at(CL2_m), DisVe5, K2[5], D2[5], KneeLength, 0, CMusclActFac);
             
             CL2Out.Pos = ParaR62.Pos;
             PreJointAng.at(CL2_m) = ParaR62.Pos;
             m_reflex_muscle.at(CL2_m) = Scaling(CL2Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
             CL2Out.Vel = ParaR62.Vel;
             PreJointVel.at(CL2_m) = ParaR62.Vel;

             old_Diff_L_fs.at(2) = Diff_L_fs.at(2);

             Diff_L_fs.at(2) = Raw_L_fs.at(2) - Exp_L_fs.at(2);

             acc_Diff_L_fs.at(2) += (-Diff_L_fs.at(2));

             deri__Diff_L_fs.at(2) =  Diff_L_fs.at(2)- old_Diff_L_fs.at(2);

             TouchL2 = TouchL2 + 1;


              AveDiff_L_fs.at(2) = (abs(Diff_L_fs.at(2)) + (TouchL2 - 1) * OldAveDiff_L_fs.at(2))/TouchL2;


             if (m_reflex_muscle.at(CL2_m) > MINCTROUT)
                     {
                       m_reflex_muscle.at(CL2_m) = MINCTROUT;
                     }

             if (LEARNINGTYPE == 9)
                  {
                    K2[5] = TSSDualRateLearning(Raw_L_fs.at(2), K2[5], Exp_L_fs.at(2),dSlowL2,dFastL2,CTRBASELINE);
                    Stiff_L.at(2) = K2[5];
                    std::cout<<"The CTr stiffness of L2 is"<<K2[5]<<"\n";

                    K1[5] = TSSDualRateLearning(Raw_L_fs.at(2), K1[5], Exp_L_fs.at(2),dFTiSlowL2,dFTiFastL2,FTIBASELINE);
                        FTiStiff_L.at(2) = K1[5];
                        std::cout<<"The FTi stiffness of R0 is"<<K1[5]<<"\n";

                        Stiff_L_Fast.at(2) = dFastL2;
                        Stiff_L_Slow.at(2) = dSlowL2;

                        FTiStiff_L_Fast.at(2) = dFTiFastL2;
                        FTiStiff_L_Slow.at(2) = dFTiSlowL2;
                  }
     
             
           }
           else
           {
             
             Exp_L_fs.at(2) = 0.0;
             m_reflex_muscle.at(FL2_m) = dSWFTiFac * m_reflex.at(FL2_m) - dSWFTiOffset;
             PreJointAng.at(FL2_m) = Scaling(m_reflex.at(FL2_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
             PreJointVel.at(FL2_m) = 0;
             m_reflex_muscle.at(CL2_m) = m_reflex.at(CL2_m);
             PreJointAng.at(CL2_m) = Scaling(m_reflex.at(CL2_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
             PreJointVel.at(CL2_m) = 0;
             
             TouchL2 = 0;
             acc_Diff_L_fs.at(2) = 0;
//
//               if (AveDiff_L_fs.at(2) > 0)
//            {
//                 dAveErrorL2 =  AveDiff_L_fs.at(2);
//
//                 if ((dAveErrorL2 > FCERROR) and (K2[5] < MAXCTRSTIFF))
//                 {
//                   K2[5] = K2[5] + CTRSTIFFSETP * dAveErrorL2;
//                   std::cout<<"The CTr stiffness of L2 is"<<K2[5]<<"\n";
//                 }
//
//            }


                            if (AveDiff_L_fs.at(2) > 0)
                         {
                              if ((LEARNINGTYPE == 6) || (LEARNINGTYPE == 8))
                               {
                                 if(AveDiff_L_fs.at(2) > LTDRLFCERROR)
                                  {
                                    dLDRLL2 = dLDRLL2 + AveDiff_L_fs.at(2);
                                  }

                               }

                              if (LEARNINGTYPE == 0)
                              {
                                K2[5] =  LMSLearning(AveDiff_L_fs.at(2), CL2_m, K2[5]);
                                Stiff_L.at(2) = K2[5];

                              }

                         }


               AveDiff_L_fs.at(2) = 0;

           }

         if (LEARNINGTYPE == 1)
         {
           K2[5] = RLSLearning(Raw_L_fs.at(2), K2[5], Exp_L_fs.at(2), dGainL2, dAutoCorL2);
           Stiff_L.at(2) = K2[5];
           std::cout<<"The CTr stiffness of L2 is"<<K2[5]<<"\n";
         }

         if (LEARNINGTYPE == 2)
         {
           K2[5] = DualRateLearning(Raw_L_fs.at(2), K2[5], Exp_L_fs.at(2));
           Stiff_L.at(2) = K2[5];
           std::cout<<"The CTr stiffness of L2 is"<<K2[5]<<"\n";
         }

         if (LEARNINGTYPE == 3)
          {
            K2[5] = TDualRateLearning(Raw_L_fs.at(2), K2[5], Exp_L_fs.at(2));
            Stiff_L.at(2) = K2[5];
            std::cout<<"The CTr stiffness of L2 is"<<K2[5]<<"\n";
          }

         if (LEARNINGTYPE == 6)
          {
            K2[5] = LTDualRateLearning(Raw_L_fs.at(2), K2[5], Exp_L_fs.at(2),dLDRLL2);
            Stiff_L.at(2) = K2[5];
            std::cout<<"The CTr stiffness of L2 is"<<K2[5]<<"\n";
          }

         if (LEARNINGTYPE == 7)
             {
               K2[5] = SDualRateLearning(Raw_L_fs.at(2), K2[5], Exp_L_fs.at(2),dSlowL2,dFastL2);
               Stiff_L.at(2) = K2[5];
               std::cout<<"The CTr stiffness of L2 is"<<K2[5]<<"\n";
             }

         if (LEARNINGTYPE == 8)
             {
               K2[5] = ParaSDualRateLearning(Raw_L_fs.at(2), K2[5], Exp_L_fs.at(2),dSlowL2,dFastL2,dLDRLL2);
               Stiff_L.at(2) = K2[5];
               std::cout<<"The CTr stiffness of L2 is"<<K2[5]<<"\n";
             }



         if (LEARNINGTYPE == 4)
          {
            K2[5] = FastLearning(Raw_L_fs.at(2), K2[5], Exp_L_fs.at(2));
            Stiff_L.at(2) = K2[5];
            std::cout<<"The CTr stiffness of L2 is"<<K2[5]<<"\n";
          }

          if (LEARNINGTYPE == 5)
           {
             K2[5] = SlowLearning(Raw_L_fs.at(2), K2[5], Exp_L_fs.at(2));
             Stiff_L.at(2) = K2[5];
             std::cout<<"The CTr stiffness of L2 is"<<K2[5]<<"\n";
           }
  
}
else if (WHMUSCLEALLPHA == 1)
{
  MusclePara CR0Out, FR0Out;

    if (m_pre.at(TR0_m) < m_preold.at(TR0_m))// && (Raw_R_fs.at(1) > FSERR)
     {
  //    m_Prereflex.at(FR0_m) = Scaling(m_Prereflex.at(FR0_m), -0.67, -1.0, 1.0, -0.9);
  //    m_Prereflex.at(CR0_m) = Scaling(m_Prereflex.at(CR0_m), 1.0, 0.71, 1.0, -1.0);
  //    m_reflex.at(FR0_m) = Scaling(m_reflex.at(FR0_m), -0.67, -1.0, 1.0, -0.9);
  //    m_reflex.at(CR0_m) = Scaling(m_reflex.at(CR0_m), 1.0, 0.71, 1.0, -1.0);

      Exp_R_fs.at(0) = 1.0;

       MusclePara ParaR11 = RungeKutta41(PreJointAng.at(FR0_m), PreJointVel.at(FR0_m), SampleTime , Exp_R_fs_old.at(0),
           Exp_R_fs.at(0), m_preold.at(FR0_m), m_pre.at(FR0_m), 0, K1[0], D1[0], HipLength, 0, FMusclActFac);

       FR0Out.Pos = ParaR11.Pos;
       PreJointAng.at(FR0_m) = ParaR11.Pos;
       m_reflex_muscle.at(FR0_m) = Scaling(FR0Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
       FR0Out.Vel = ParaR11.Vel;
       PreJointVel.at(FR0_m) =  ParaR11.Vel;

       double DisVe = (HipLength + Radius) * sin(ParaR11.Pos);
  //     MusclePara ParaR12 = RungeKutta42(PreJointAng.at(CR0_m), PreJointVel.at(CR0_m), SampleTime , Raw_R_fs_old.at(0),
  //         Raw_R_fs.at(0), m_Prereflex.at(CR0_m), m_reflex.at(CR0_m), DisVe, K2[0], D2[0], KneeLength, 0, CMusclActFac);

       MusclePara ParaR12 = RungeKutta42(PreJointAng.at(CR0_m), PreJointVel.at(CR0_m), SampleTime , Exp_R_fs_old.at(0),
           Exp_R_fs.at(0), m_preold.at(CR0_m), m_pre.at(CR0_m), DisVe, K2[0], D2[0], KneeLength, 0, CMusclActFac);

       CR0Out.Pos = ParaR12.Pos;
       PreJointAng.at(CR0_m) = ParaR12.Pos;
       m_reflex_muscle.at(CR0_m) = Scaling(CR0Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
       CR0Out.Vel = ParaR12.Vel;
       PreJointVel.at(CR0_m) = ParaR12.Vel;

       Diff_R_fs.at(0) = Raw_R_fs.at(0) - Exp_R_fs.at(0);

       TouchR0 = TouchR0 + 1;


        AveDiff_R_fs.at(0) = (abs(Diff_R_fs.at(0)) + (TouchR0 - 1) * OldAveDiff_R_fs.at(0))/TouchR0;



        if (m_reflex_muscle.at(CR0_m) > MINCTROUT)
         {
           m_reflex_muscle.at(CR0_m) = MINCTROUT;
         }


     }
     else
     {
       Exp_R_fs.at(0) = 0.0;

       m_reflex_muscle.at(FR0_m) = dSWFTiFac * m_reflex.at(FR0_m) - dSWFTiOffset;
       PreJointAng.at(FR0_m) = Scaling(m_reflex.at(FR0_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
       PreJointVel.at(FR0_m) = 0;
       m_reflex_muscle.at(CR0_m) = m_reflex.at(CR0_m);
       PreJointAng.at(CR0_m) = Scaling(m_reflex.at(CR0_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
       PreJointVel.at(CR0_m) = 0;

       TouchR0 = 0;
       if (AveDiff_R_fs.at(0) > 0)
    {
         dAveErrorR0 =  AveDiff_R_fs.at(0);

    }
       AveDiff_R_fs.at(0) = 0;

     }

    MusclePara CR1Out, FR1Out;

     if (m_pre.at(TR1_m) < m_preold.at(TR1_m))// && (Raw_R_fs.at(1) > FSERR)
      {
  //     m_Prereflex.at(FR1_m) = Scaling(m_Prereflex.at(FR1_m), -0.67, -1.0, 1.0, -0.9);
  //     m_reflex.at(FR1_m) = Scaling(m_reflex.at(FR1_m), -0.67, -1.0, 1.0, -0.9);
  //     m_Prereflex.at(CR1_m) = Scaling(m_Prereflex.at(CR1_m), 1.0, 0.71, 1.0, -1.0);
  //     m_reflex.at(CR1_m) = Scaling(m_reflex.at(CR1_m), 1.0, 0.71, 1.0, -1.0);

       Exp_R_fs.at(1) = 1.0;
        MusclePara ParaR21 = RungeKutta41(PreJointAng.at(FR1_m), PreJointVel.at(FR1_m), SampleTime , Exp_R_fs_old.at(1),
            Exp_R_fs.at(1), m_preold.at(FR1_m), m_pre.at(FR1_m), 0, K1[1], D1[1], HipLength, 0, FMusclActFac);

        FR1Out.Pos = ParaR21.Pos;
        PreJointAng.at(FR1_m) = ParaR21.Pos;
        m_reflex_muscle.at(FR1_m) = Scaling(FR1Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
        FR1Out.Vel = ParaR21.Vel;
        PreJointVel.at(FR1_m) = ParaR21.Vel;

        double DisVe1 = (HipLength + Radius) * sin(ParaR21.Pos);
  //      MusclePara ParaR22 = RungeKutta42(PreJointAng.at(CR1_m), PreJointVel.at(CR1_m), SampleTime , Raw_R_fs_old.at(1),
  //          Raw_R_fs.at(1), m_Prereflex.at(CR1_m), m_reflex.at(CR1_m), DisVe1, K2[1], D2[1], KneeLength, 0, CMusclActFac);

        MusclePara ParaR22 = RungeKutta42(PreJointAng.at(CR1_m), PreJointVel.at(CR1_m), SampleTime , Exp_R_fs_old.at(1),
            Exp_R_fs.at(1),  m_preold.at(CR1_m), m_pre.at(CR1_m), DisVe1, K2[1], D2[1], KneeLength, 0, CMusclActFac);

        CR1Out.Pos = ParaR22.Pos;
        PreJointAng.at(CR1_m) = ParaR22.Pos;
        m_reflex_muscle.at(CR1_m) = Scaling(CR1Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
        CR1Out.Vel = ParaR22.Vel;
        PreJointVel.at(CR1_m) = ParaR22.Vel;

        if (m_reflex_muscle.at(CR1_m) > MINCTROUT)
            {
              m_reflex_muscle.at(CR1_m) = MINCTROUT;
            }


      }
      else
      {
        Exp_R_fs.at(1) = 0.0;

        m_reflex_muscle.at(FR1_m) = dSWFTiFac * m_reflex.at(FR1_m) - dSWFTiOffset;
        PreJointAng.at(FR1_m) = Scaling(m_reflex.at(FR1_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
        PreJointVel.at(FR1_m) = 0;
        m_reflex_muscle.at(CR1_m) = m_reflex.at(CR1_m);
        PreJointAng.at(CR1_m) = Scaling(m_reflex.at(CR1_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
        PreJointVel.at(CR1_m) = 0;

      }

     MusclePara CR2Out, FR2Out;

      if (m_pre.at(TR2_m) < m_preold.at(TR2_m))// && (Raw_R_fs.at(1) > FSERR)
       {

  //      m_Prereflex.at(FR2_m) = Scaling(m_Prereflex.at(FR2_m), -0.67, -1.0, 1.0, -0.9);
  //      m_reflex.at(FR2_m) = Scaling(m_reflex.at(FR2_m), -0.67, -1.0, 1.0, -0.9);
  //      m_Prereflex.at(CR2_m) = Scaling(m_Prereflex.at(CR2_m), 1.0, 0.71, 1.0, -1.0);
  //      m_reflex.at(CR2_m) = Scaling(m_reflex.at(CR2_m), 1.0, 0.71, 1.0, -1.0);
        Exp_R_fs.at(2) = 1.0;
         MusclePara ParaR31 = RungeKutta41(PreJointAng.at(FR2_m), PreJointVel.at(FR2_m), SampleTime , Exp_R_fs_old.at(2),
             Exp_R_fs.at(2), m_preold.at(FR2_m), m_pre.at(FR2_m), 0, K1[2], D1[2], HipLength, 0, FMusclActFac);

         FR2Out.Pos = ParaR31.Pos;
         PreJointAng.at(FR2_m) = ParaR31.Pos;
         m_reflex_muscle.at(FR2_m) = Scaling(FR2Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
         FR2Out.Vel = ParaR31.Vel;
         PreJointVel.at(FR2_m) = ParaR31.Vel;

         double DisVe2 = (HipLength + Radius) * sin(ParaR31.Pos);
  //       MusclePara ParaR32 = RungeKutta42(PreJointAng.at(CR2_m), PreJointVel.at(CR2_m), SampleTime , Raw_R_fs_old.at(2),
  //           Raw_R_fs.at(2), m_Prereflex.at(CR2_m), m_reflex.at(CR2_m), DisVe2, K2[2], D2[2], KneeLength, 0, CMusclActFac);

         MusclePara ParaR32 = RungeKutta42(PreJointAng.at(CR2_m), PreJointVel.at(CR2_m), SampleTime , Exp_R_fs_old.at(2),
             Exp_R_fs.at(2), m_preold.at(CR2_m), m_pre.at(CR2_m), DisVe2, K2[2], D2[2], KneeLength, 0, CMusclActFac);


         CR2Out.Pos = ParaR32.Pos;
         PreJointAng.at(CR2_m) = ParaR32.Pos;
         m_reflex_muscle.at(CR2_m) = Scaling(CR2Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
         CR2Out.Vel = ParaR32.Vel;
         PreJointVel.at(CR2_m) = ParaR32.Vel;

         if (m_reflex_muscle.at(CR2_m) > MINCTROUT)
             {
               m_reflex_muscle.at(CR2_m) = MINCTROUT;
             }


       }
       else
       {
         Exp_R_fs.at(2) = 0.0;

         m_reflex_muscle.at(FR2_m) = dSWFTiFac * m_reflex.at(FR2_m) - dSWFTiOffset;
         PreJointAng.at(FR2_m) = Scaling(m_reflex.at(FR2_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
         PreJointVel.at(FR2_m) = 0;
         m_reflex_muscle.at(CR2_m) = m_reflex.at(CR2_m);
         PreJointAng.at(CR2_m) = Scaling(m_reflex.at(CR2_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
         PreJointVel.at(CR2_m) = 0;

       }


      MusclePara CL0Out, FL0Out;

       if (m_pre.at(TL0_m) < m_preold.at(TL0_m))// && (Raw_R_fs.at(1) > FSERR)
        {

  //       m_Prereflex.at(FL0_m) = Scaling(m_Prereflex.at(FL0_m), -0.67, -1.0, 1.0, -0.9);
  //       m_reflex.at(FL0_m) = Scaling(m_reflex.at(FL0_m), -0.67, -1.0, 1.0, -0.9);
  //       m_Prereflex.at(CL0_m) = Scaling(m_Prereflex.at(CL0_m), 1.0, 0.71, 1.0, -1.0);
  //       m_reflex.at(CL0_m) = Scaling(m_reflex.at(CL0_m), 1.0, 0.71, 1.0, -1.0);
  //
         Exp_L_fs.at(0) = 1.0;
           MusclePara ParaR41 = RungeKutta41(PreJointAng.at(FL0_m), PreJointVel.at(FL0_m), SampleTime , Exp_L_fs_old.at(0),
               Exp_L_fs.at(0), m_preold.at(FL0_m), m_pre.at(FL0_m), 0, K1[3], D1[3], HipLength, 0, FMusclActFac);

           FL0Out.Pos = ParaR41.Pos;
           PreJointAng.at(FL0_m) = ParaR41.Pos;
           m_reflex_muscle.at(FL0_m) = Scaling(FL0Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
           FL0Out.Vel = ParaR41.Vel;
           PreJointVel.at(FL0_m) = ParaR41.Vel;

           double DisVe3 = (HipLength + Radius) * sin(ParaR41.Pos);
  //         MusclePara ParaR42 = RungeKutta42(PreJointAng.at(CL0_m), PreJointVel.at(CL0_m), SampleTime , Raw_L_fs_old.at(0),
  //             Raw_L_fs.at(0), m_Prereflex.at(CL0_m), m_reflex.at(CL0_m), DisVe3, K2[3], D2[3], KneeLength, 0, CMusclActFac);

           MusclePara ParaR42 = RungeKutta42(PreJointAng.at(CL0_m), PreJointVel.at(CL0_m), SampleTime , Exp_L_fs_old.at(0),
               Exp_L_fs.at(0), m_preold.at(CL0_m), m_pre.at(CL0_m), DisVe3, K2[3], D2[3], KneeLength, 0, CMusclActFac);

           CL0Out.Pos = ParaR42.Pos;
           PreJointAng.at(CL0_m) = ParaR42.Pos;
           m_reflex_muscle.at(CL0_m) = Scaling(CL0Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
           CL0Out.Vel = ParaR42.Vel;
           PreJointVel.at(CL0_m) = ParaR42.Vel;

           if (m_reflex_muscle.at(CL0_m) > MINCTROUT)
                   {
                     m_reflex_muscle.at(CL0_m) = MINCTROUT;
                   }


         }
         else
         {

           Exp_L_fs.at(0) = 0.0;
           m_reflex_muscle.at(FL0_m) = dSWFTiFac * m_reflex.at(FL0_m) - dSWFTiOffset;
           PreJointAng.at(FL0_m) = Scaling(m_reflex.at(FL0_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
           PreJointVel.at(FL0_m) = 0;
           m_reflex_muscle.at(CL0_m) = m_reflex.at(CL0_m);
           PreJointAng.at(CL0_m) = Scaling(m_reflex.at(CL0_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
           PreJointVel.at(CL0_m) = 0;

         }

       MusclePara CL1Out, FL1Out;

        if (m_pre.at(TL1_m) < m_preold.at(TL1_m))// && (Raw_R_fs.at(1) > FSERR)
         {
  //        m_Prereflex.at(FL1_m) = Scaling(m_Prereflex.at(FL1_m), -0.67, -1.0, 1.0, -0.9);
  //        m_reflex.at(FL1_m) = Scaling(m_reflex.at(FL1_m), -0.67, -1.0, 1.0, -0.9);
  //        m_Prereflex.at(CL1_m) = Scaling(m_Prereflex.at(CL1_m), 1.0, 0.71, 1.0, -1.0);
  //        m_reflex.at(CL1_m) = Scaling(m_reflex.at(CL1_m), 1.0, 0.71, 1.0, -1.0);


            Exp_L_fs.at(1) = 1.0;
            MusclePara ParaR51 = RungeKutta41(PreJointAng.at(FL1_m), PreJointVel.at(FL1_m), SampleTime , Exp_L_fs_old.at(1),
                Exp_L_fs.at(1), m_preold.at(FL1_m), m_pre.at(FL1_m), 0, K1[4], D1[4], HipLength, 0, FMusclActFac);

            FL1Out.Pos = ParaR51.Pos;
            PreJointAng.at(FL1_m) = ParaR51.Pos;
            m_reflex_muscle.at(FL1_m) = Scaling(FL1Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
            FL1Out.Vel = ParaR51.Vel;
            PreJointVel.at(FL1_m) = ParaR51.Vel;

            double DisVe4 = (HipLength + Radius) * sin(ParaR51.Pos);
  //          MusclePara ParaR52 = RungeKutta42(PreJointAng.at(CL1_m), PreJointVel.at(CL1_m), SampleTime , Raw_L_fs_old.at(1),
  //              Raw_L_fs.at(1), m_Prereflex.at(CL1_m), m_reflex.at(CL1_m), DisVe4, K2[4], D2[4], KneeLength, 0, CMusclActFac);

            MusclePara ParaR52 = RungeKutta42(PreJointAng.at(CL1_m), PreJointVel.at(CL1_m), SampleTime , Exp_L_fs_old.at(1),
                Exp_L_fs.at(1), m_preold.at(CL1_m), m_pre.at(CL1_m), DisVe4, K2[4], D2[4], KneeLength, 0, CMusclActFac);

            CL1Out.Pos = ParaR52.Pos;
            PreJointAng.at(CL1_m) = ParaR52.Pos;
            m_reflex_muscle.at(CL1_m) = Scaling(CL1Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
            CL1Out.Vel = ParaR52.Vel;
            PreJointVel.at(CL1_m) = ParaR52.Vel;

            if (m_reflex_muscle.at(CL1_m) > MINCTROUT)
                    {
                      m_reflex_muscle.at(CL1_m) = MINCTROUT;
                    }


          }
          else
          {

            Exp_L_fs.at(1) = 0.0;
            m_reflex_muscle.at(FL1_m) = dSWFTiFac * m_reflex.at(FL1_m) - dSWFTiOffset;
            PreJointAng.at(FL1_m) = Scaling(m_reflex.at(FL1_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
            PreJointVel.at(FL1_m) = 0;
            m_reflex_muscle.at(CL1_m) = m_reflex.at(CL1_m);
            PreJointAng.at(CL1_m) = Scaling(m_reflex.at(CL1_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
            PreJointVel.at(CL1_m) = 0;

          }

        MusclePara CL2Out, FL2Out;

           if (m_pre.at(TL2_m) < m_preold.at(TL2_m))// && (Raw_R_fs.at(1) > FSERR)
            {

  //           m_Prereflex.at(CL2_m) = Scaling(m_Prereflex.at(CL2_m), 1.0, 0.71, 1.0, -1.0);
  //           m_Prereflex.at(FL2_m) = Scaling(m_Prereflex.at(FL2_m), -0.67, -1.0, 1.0, -0.9);
  //           m_reflex.at(FL2_m) = Scaling(m_reflex.at(FL2_m), -0.67, -1.0, 1.0, -0.9);
  //           m_reflex.at(CL2_m) = Scaling(m_reflex.at(CL2_m), 1.0, 0.71, 1.0, -1.0);
             Exp_L_fs.at(2) = 1.0;
               MusclePara ParaR61 = RungeKutta41(PreJointAng.at(FL2_m), PreJointVel.at(FL2_m), SampleTime , Exp_L_fs_old.at(2),
                   Exp_L_fs.at(2), m_preold.at(FL2_m), m_pre.at(FL2_m), 0, K1[5], D1[5], HipLength, 0, FMusclActFac);

               FL2Out.Pos = ParaR61.Pos;
               PreJointAng.at(FL2_m) = ParaR61.Pos;
               m_reflex_muscle.at(FL2_m) = Scaling(FL2Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
               FL2Out.Vel = ParaR61.Vel;
               PreJointVel.at(FL2_m) = ParaR61.Vel;

               double DisVe5 = (HipLength + Radius) * sin(ParaR61.Pos);
  //             MusclePara ParaR62 = RungeKutta42(PreJointAng.at(CL2_m), PreJointVel.at(CL2_m), SampleTime , Raw_L_fs_old.at(2),
  //                 Raw_L_fs.at(2), m_Prereflex.at(CL2_m), m_reflex.at(CL2_m), DisVe5, K2[5], D2[5], KneeLength, 0, CMusclActFac);

               MusclePara ParaR62 = RungeKutta42(PreJointAng.at(CL2_m), PreJointVel.at(CL2_m), SampleTime , Exp_L_fs_old.at(2),
                   Exp_L_fs.at(2), m_preold.at(CL2_m), m_pre.at(CL2_m), DisVe5, K2[5], D2[5], KneeLength, 0, CMusclActFac);

               CL2Out.Pos = ParaR62.Pos;
               PreJointAng.at(CL2_m) = ParaR62.Pos;
               m_reflex_muscle.at(CL2_m) = Scaling(CL2Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
               CL2Out.Vel = ParaR62.Vel;
               PreJointVel.at(CL2_m) = ParaR62.Vel;

               if (m_reflex_muscle.at(CL2_m) > MINCTROUT)
                       {
                         m_reflex_muscle.at(CL2_m) = MINCTROUT;
                       }


             }
             else
             {

               Exp_L_fs.at(2) = 0.0;
               m_reflex_muscle.at(FL2_m) = dSWFTiFac * m_reflex.at(FL2_m) - dSWFTiOffset;
               PreJointAng.at(FL2_m) = Scaling(m_reflex.at(FL2_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
               PreJointVel.at(FL2_m) = 0;
               m_reflex_muscle.at(CL2_m) = m_reflex.at(CL2_m);
               PreJointAng.at(CL2_m) = Scaling(m_reflex.at(CL2_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
               PreJointVel.at(CL2_m) = 0;

             }

}
else
{
  


  
  MusclePara CR0Out, FR0Out;

//    m_Prereflex.at(FR0_m) = Scaling(m_Prereflex.at(FR0_m), -0.67, -1.0, 1.0, -0.9);
//    m_Prereflex.at(CR0_m) = Scaling(m_Prereflex.at(CR0_m), 1.0, 0.71, 1.0, -1.0);
//    m_reflex.at(FR0_m) = Scaling(m_reflex.at(FR0_m), -0.67, -1.0, 1.0, -0.9);
//    m_reflex.at(CR0_m) = Scaling(m_reflex.at(CR0_m), 1.0, 0.71, 1.0, -1.0);
    
     MusclePara ParaR11 = RungeKutta41(PreJointAng.at(FR0_m), PreJointVel.at(FR0_m), SampleTime , Raw_R_fs_old.at(0), 
         Raw_R_fs.at(0), m_preold.at(FR0_m), m_pre.at(FR0_m), 0, K1[0], D1[0], HipLength, 0, FMusclActFac);
     
     FR0Out.Pos = ParaR11.Pos;
     PreJointAng.at(FR0_m) = ParaR11.Pos;
     m_reflex_muscle.at(FR0_m) = Scaling(FR0Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
     FR0Out.Vel = ParaR11.Vel;
     PreJointVel.at(FR0_m) =  ParaR11.Vel;
     
     double DisVe = (HipLength + Radius) * sin(ParaR11.Pos);
//     MusclePara ParaR12 = RungeKutta42(PreJointAng.at(CR0_m), PreJointVel.at(CR0_m), SampleTime , Raw_R_fs_old.at(0), 
//         Raw_R_fs.at(0), m_Prereflex.at(CR0_m), m_reflex.at(CR0_m), DisVe, K2[0], D2[0], KneeLength, 0, CMusclActFac);
     
     MusclePara ParaR12 = RungeKutta42(PreJointAng.at(CR0_m), PreJointVel.at(CR0_m), SampleTime , Raw_R_fs_old.at(0), 
         Raw_R_fs.at(0), m_preold.at(CR0_m), m_pre.at(CR0_m), DisVe, K2[0], D2[0], KneeLength, 0, CMusclActFac);
     
     CR0Out.Pos = ParaR12.Pos;
     PreJointAng.at(CR0_m) = ParaR12.Pos;
     m_reflex_muscle.at(CR0_m) = Scaling(CR0Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
     CR0Out.Vel = ParaR12.Vel;
     PreJointVel.at(CR0_m) = ParaR12.Vel;
     

  MusclePara CR1Out, FR1Out;

//     m_Prereflex.at(FR1_m) = Scaling(m_Prereflex.at(FR1_m), -0.67, -1.0, 1.0, -0.9);
//     m_reflex.at(FR1_m) = Scaling(m_reflex.at(FR1_m), -0.67, -1.0, 1.0, -0.9);
//     m_Prereflex.at(CR1_m) = Scaling(m_Prereflex.at(CR1_m), 1.0, 0.71, 1.0, -1.0);
//     m_reflex.at(CR1_m) = Scaling(m_reflex.at(CR1_m), 1.0, 0.71, 1.0, -1.0);
      
      MusclePara ParaR21 = RungeKutta41(PreJointAng.at(FR1_m), PreJointVel.at(FR1_m), SampleTime , Raw_R_fs_old.at(1), 
          Raw_R_fs.at(1), m_preold.at(FR1_m), m_pre.at(FR1_m), 0, K1[1], D1[1], HipLength, 0, FMusclActFac);
      
      FR1Out.Pos = ParaR21.Pos;
      PreJointAng.at(FR1_m) = ParaR21.Pos;
      m_reflex_muscle.at(FR1_m) = Scaling(FR1Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
      FR1Out.Vel = ParaR21.Vel;
      PreJointVel.at(FR1_m) = ParaR21.Vel;
                  
      double DisVe1 = (HipLength + Radius) * sin(ParaR21.Pos);
//      MusclePara ParaR22 = RungeKutta42(PreJointAng.at(CR1_m), PreJointVel.at(CR1_m), SampleTime , Raw_R_fs_old.at(1), 
//          Raw_R_fs.at(1), m_Prereflex.at(CR1_m), m_reflex.at(CR1_m), DisVe1, K2[1], D2[1], KneeLength, 0, CMusclActFac);
      
      MusclePara ParaR22 = RungeKutta42(PreJointAng.at(CR1_m), PreJointVel.at(CR1_m), SampleTime , Raw_R_fs_old.at(1), 
          Raw_R_fs.at(1),  m_preold.at(CR1_m), m_pre.at(CR1_m), DisVe1, K2[1], D2[1], KneeLength, 0, CMusclActFac);
      
      CR1Out.Pos = ParaR22.Pos;
      PreJointAng.at(CR1_m) = ParaR22.Pos;
      m_reflex_muscle.at(CR1_m) = Scaling(CR1Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
      CR1Out.Vel = ParaR22.Vel;
      PreJointVel.at(CR1_m) = ParaR22.Vel;


   MusclePara CR2Out, FR2Out;

      
//      m_Prereflex.at(FR2_m) = Scaling(m_Prereflex.at(FR2_m), -0.67, -1.0, 1.0, -0.9);
//      m_reflex.at(FR2_m) = Scaling(m_reflex.at(FR2_m), -0.67, -1.0, 1.0, -0.9);
//      m_Prereflex.at(CR2_m) = Scaling(m_Prereflex.at(CR2_m), 1.0, 0.71, 1.0, -1.0);
//      m_reflex.at(CR2_m) = Scaling(m_reflex.at(CR2_m), 1.0, 0.71, 1.0, -1.0);
       
       MusclePara ParaR31 = RungeKutta41(PreJointAng.at(FR2_m), PreJointVel.at(FR2_m), SampleTime , Raw_R_fs_old.at(2), 
           Raw_R_fs.at(2), m_preold.at(FR2_m), m_pre.at(FR2_m), 0, K1[2], D1[2], HipLength, 0, FMusclActFac);
       
       FR2Out.Pos = ParaR31.Pos;
       PreJointAng.at(FR2_m) = ParaR31.Pos;
       m_reflex_muscle.at(FR2_m) = Scaling(FR2Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
       FR2Out.Vel = ParaR31.Vel;
       PreJointVel.at(FR2_m) = ParaR31.Vel;
                   
       double DisVe2 = (HipLength + Radius) * sin(ParaR31.Pos);
//       MusclePara ParaR32 = RungeKutta42(PreJointAng.at(CR2_m), PreJointVel.at(CR2_m), SampleTime , Raw_R_fs_old.at(2), 
//           Raw_R_fs.at(2), m_Prereflex.at(CR2_m), m_reflex.at(CR2_m), DisVe2, K2[2], D2[2], KneeLength, 0, CMusclActFac);
       
       MusclePara ParaR32 = RungeKutta42(PreJointAng.at(CR2_m), PreJointVel.at(CR2_m), SampleTime , Raw_R_fs_old.at(2), 
                Raw_R_fs.at(2), m_preold.at(CR2_m), m_pre.at(CR2_m), DisVe2, K2[2], D2[2], KneeLength, 0, CMusclActFac);
            
       
       CR2Out.Pos = ParaR32.Pos;
       PreJointAng.at(CR2_m) = ParaR32.Pos;
       m_reflex_muscle.at(CR2_m) = Scaling(CR2Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
       CR2Out.Vel = ParaR32.Vel;
       PreJointVel.at(CR2_m) = ParaR32.Vel;

    
    
    MusclePara CL0Out, FL0Out;

//       m_Prereflex.at(FL0_m) = Scaling(m_Prereflex.at(FL0_m), -0.67, -1.0, 1.0, -0.9);
//       m_reflex.at(FL0_m) = Scaling(m_reflex.at(FL0_m), -0.67, -1.0, 1.0, -0.9);
//       m_Prereflex.at(CL0_m) = Scaling(m_Prereflex.at(CL0_m), 1.0, 0.71, 1.0, -1.0);
//       m_reflex.at(CL0_m) = Scaling(m_reflex.at(CL0_m), 1.0, 0.71, 1.0, -1.0);
//         
         MusclePara ParaR41 = RungeKutta41(PreJointAng.at(FL0_m), PreJointVel.at(FL0_m), SampleTime , Raw_L_fs_old.at(0), 
             Raw_L_fs.at(0), m_preold.at(FL0_m), m_pre.at(FL0_m), 0, K1[3], D1[3], HipLength, 0, FMusclActFac);
         
         FL0Out.Pos = ParaR41.Pos;
         PreJointAng.at(FL0_m) = ParaR41.Pos;
         m_reflex_muscle.at(FL0_m) = Scaling(FL0Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
         FL0Out.Vel = ParaR41.Vel;
         PreJointVel.at(FL0_m) = ParaR41.Vel;
                     
         double DisVe3 = (HipLength + Radius) * sin(ParaR41.Pos);
//         MusclePara ParaR42 = RungeKutta42(PreJointAng.at(CL0_m), PreJointVel.at(CL0_m), SampleTime , Raw_L_fs_old.at(0), 
//             Raw_L_fs.at(0), m_Prereflex.at(CL0_m), m_reflex.at(CL0_m), DisVe3, K2[3], D2[3], KneeLength, 0, CMusclActFac);
         
         MusclePara ParaR42 = RungeKutta42(PreJointAng.at(CL0_m), PreJointVel.at(CL0_m), SampleTime , Raw_L_fs_old.at(0), 
             Raw_L_fs.at(0), m_preold.at(CL0_m), m_pre.at(CL0_m), DisVe3, K2[3], D2[3], KneeLength, 0, CMusclActFac);
         
         CL0Out.Pos = ParaR42.Pos;
         PreJointAng.at(CL0_m) = ParaR42.Pos;
         m_reflex_muscle.at(CL0_m) = Scaling(CL0Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
         CL0Out.Vel = ParaR42.Vel;
         PreJointVel.at(CL0_m) = ParaR42.Vel;

         
  
     
     MusclePara CL1Out, FL1Out;
       
     
//        m_Prereflex.at(FL1_m) = Scaling(m_Prereflex.at(FL1_m), -0.67, -1.0, 1.0, -0.9);
//        m_reflex.at(FL1_m) = Scaling(m_reflex.at(FL1_m), -0.67, -1.0, 1.0, -0.9);
//        m_Prereflex.at(CL1_m) = Scaling(m_Prereflex.at(CL1_m), 1.0, 0.71, 1.0, -1.0);
//        m_reflex.at(CL1_m) = Scaling(m_reflex.at(CL1_m), 1.0, 0.71, 1.0, -1.0);
          
          MusclePara ParaR51 = RungeKutta41(PreJointAng.at(FL1_m), PreJointVel.at(FL1_m), SampleTime , Raw_L_fs_old.at(1), 
              Raw_L_fs.at(1), m_preold.at(FL1_m), m_pre.at(FL1_m), 0, K1[4], D1[4], HipLength, 0, FMusclActFac);
          
          FL1Out.Pos = ParaR51.Pos;
          PreJointAng.at(FL1_m) = ParaR51.Pos;
          m_reflex_muscle.at(FL1_m) = Scaling(FL1Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
          FL1Out.Vel = ParaR51.Vel;
          PreJointVel.at(FL1_m) = ParaR51.Vel;
                      
          double DisVe4 = (HipLength + Radius) * sin(ParaR51.Pos);
//          MusclePara ParaR52 = RungeKutta42(PreJointAng.at(CL1_m), PreJointVel.at(CL1_m), SampleTime , Raw_L_fs_old.at(1), 
//              Raw_L_fs.at(1), m_Prereflex.at(CL1_m), m_reflex.at(CL1_m), DisVe4, K2[4], D2[4], KneeLength, 0, CMusclActFac);
          
          MusclePara ParaR52 = RungeKutta42(PreJointAng.at(CL1_m), PreJointVel.at(CL1_m), SampleTime , Raw_L_fs_old.at(1), 
              Raw_L_fs.at(1), m_preold.at(CL1_m), m_pre.at(CL1_m), DisVe4, K2[4], D2[4], KneeLength, 0, CMusclActFac);
          
          CL1Out.Pos = ParaR52.Pos;
          PreJointAng.at(CL1_m) = ParaR52.Pos;
          m_reflex_muscle.at(CL1_m) = Scaling(CL1Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
          CL1Out.Vel = ParaR52.Vel;
          PreJointVel.at(CL1_m) = ParaR52.Vel;
  
          
  
      MusclePara CL2Out, FL2Out;
          
 
//           m_Prereflex.at(CL2_m) = Scaling(m_Prereflex.at(CL2_m), 1.0, 0.71, 1.0, -1.0);
//           m_Prereflex.at(FL2_m) = Scaling(m_Prereflex.at(FL2_m), -0.67, -1.0, 1.0, -0.9);
//           m_reflex.at(FL2_m) = Scaling(m_reflex.at(FL2_m), -0.67, -1.0, 1.0, -0.9);
//           m_reflex.at(CL2_m) = Scaling(m_reflex.at(CL2_m), 1.0, 0.71, 1.0, -1.0);
             
             MusclePara ParaR61 = RungeKutta41(PreJointAng.at(FL2_m), PreJointVel.at(FL2_m), SampleTime , Raw_L_fs_old.at(2), 
                 Raw_L_fs.at(2), m_preold.at(FL2_m), m_pre.at(FL2_m), 0, K1[5], D1[5], HipLength, 0, FMusclActFac);
             
             FL2Out.Pos = ParaR61.Pos;
             PreJointAng.at(FL2_m) = ParaR61.Pos;
             m_reflex_muscle.at(FL2_m) = Scaling(FL2Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
             FL2Out.Vel = ParaR61.Vel;
             PreJointVel.at(FL2_m) = ParaR61.Vel;
                         
             double DisVe5 = (HipLength + Radius) * sin(ParaR61.Pos);
//             MusclePara ParaR62 = RungeKutta42(PreJointAng.at(CL2_m), PreJointVel.at(CL2_m), SampleTime , Raw_L_fs_old.at(2), 
//                 Raw_L_fs.at(2), m_Prereflex.at(CL2_m), m_reflex.at(CL2_m), DisVe5, K2[5], D2[5], KneeLength, 0, CMusclActFac);
             
             MusclePara ParaR62 = RungeKutta42(PreJointAng.at(CL2_m), PreJointVel.at(CL2_m), SampleTime , Raw_L_fs_old.at(2), 
                              Raw_L_fs.at(2), m_preold.at(CL2_m), m_pre.at(CL2_m), DisVe5, K2[5], D2[5], KneeLength, 0, CMusclActFac);
             
             CL2Out.Pos = ParaR62.Pos;
             PreJointAng.at(CL2_m) = ParaR62.Pos;
             m_reflex_muscle.at(CL2_m) = Scaling(CL2Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
             CL2Out.Vel = ParaR62.Vel;
             PreJointVel.at(CL2_m) = ParaR62.Vel;
     
  
  
}

gettimeofday(&NowTime, NULL);//time(&NowTime);
dTimeDiff = (NowTime.tv_sec - StartTime.tv_sec) + (NowTime.tv_usec - StartTime.tv_usec)/1000000.0;//difftime(NowTime,StartTime);

if (ISRECORD == 1)
  {
    oCTrStiff[0]<<K2[0]<<'\n';
    oCTrStiff[1]<<K2[1]<<'\n';
    oCTrStiff[2]<<K2[2]<<'\n';
    oCTrStiff[3]<<K2[3]<<'\n';
    oCTrStiff[4]<<K2[4]<<'\n';
    oCTrStiff[5]<<K2[5]<<'\n';
    oRealTime<<dTimeDiff<<'\n';
  }

//std::cout<<time(NULL)<<'\n';

    
Diff_R_fs.at(0) = Raw_R_fs.at(0) - Exp_R_fs.at(0);
Diff_R_fs.at(1) = Raw_R_fs.at(1) - Exp_R_fs.at(1);
Diff_R_fs.at(2) = Raw_R_fs.at(2) - Exp_R_fs.at(2);

Diff_L_fs.at(0) = Raw_L_fs.at(0) - Exp_L_fs.at(0);
Diff_L_fs.at(1) = Raw_L_fs.at(1) - Exp_L_fs.at(1);
Diff_L_fs.at(2) = Raw_L_fs.at(2) - Exp_L_fs.at(2);



  

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
    
    if (PURENEURON == 1)
    {
      m.at(TR0_m) = m_reflex.at(TR0_m);//*1.2-0.3;
       m.at(TR1_m) = m_reflex.at(TR1_m);//*1.2+0.2;
       m.at(TR2_m) = m_reflex.at(TR2_m);//*1.2+0.5;

       m.at(TL0_m) = m_reflex.at(TL0_m);//*1.2-0.3;
       m.at(TL1_m) = m_reflex.at(TL1_m);//*1.2+0.2;
       m.at(TL2_m) = m_reflex.at(TL2_m);//*1.2+0.5;

       m.at(CL0_m) = m_reflex.at(CL0_m);
       m.at(CL1_m) = m_reflex.at(CL1_m);
       m.at(CL2_m) = m_reflex.at(CL2_m);
   
       m.at(CR0_m) = m_reflex.at(CR0_m);
       m.at(CR1_m) = m_reflex.at(CR1_m);
       m.at(CR2_m) = m_reflex.at(CR2_m);
   
   
       m.at(FR0_m) = m_reflex.at(FR0_m);
       m.at(FR1_m) = m_reflex.at(FR1_m);
       m.at(FR2_m) = m_reflex.at(FR2_m);
   
       m.at(FL0_m) = m_reflex.at(FL0_m);
       m.at(FL1_m) = m_reflex.at(FL1_m);
       m.at(FL2_m) = m_reflex.at(FL2_m);
       
  
    }
    else
    {
      m.at(TR0_m) = m_reflex.at(TR0_m);//*1.2-0.3;
      m.at(TR1_m) = m_reflex.at(TR1_m);//*1.2+0.2;
      m.at(TR2_m) = m_reflex.at(TR2_m);//*1.2+0.5;

      m.at(TL0_m) = m_reflex.at(TL0_m);//*1.2-0.3;
      m.at(TL1_m) = m_reflex.at(TL1_m);//*1.2+0.2;
      m.at(TL2_m) = m_reflex.at(TL2_m);//*1.2+0.5;



      m.at(CL0_m) = m_reflex_muscle.at(CL0_m);
      m.at(CL1_m) = m_reflex_muscle.at(CL1_m);
      m.at(CL2_m) = m_reflex_muscle.at(CL2_m);

      m.at(CR0_m) = m_reflex_muscle.at(CR0_m);
      m.at(CR1_m) = m_reflex_muscle.at(CR1_m);
      m.at(CR2_m) = m_reflex_muscle.at(CR2_m);



      //Left FTi
            if (m_reflex_muscle.at(FL0_m) < MINFTIOUT)
            {
              m_reflex_muscle.at(FL0_m) = MINFTIOUT;
            }
            if (m_reflex_muscle.at(FL1_m) < MINFTIOUT)
            {
            m_reflex_muscle.at(FL1_m) = MINFTIOUT;
            }
            if (m_reflex_muscle.at(FL2_m) < MINFTIOUT)
            {
              m_reflex_muscle.at(FL2_m) = MINFTIOUT;
            }

            //Right FTi

            if (m_reflex_muscle.at(FR0_m) < MINFTIOUT)
            {
            m_reflex_muscle.at(FR0_m) = MINFTIOUT;
            }
            if (m_reflex_muscle.at(FR1_m) < MINFTIOUT)
            {
             m_reflex_muscle.at(FR1_m) = MINFTIOUT;
            }
            if (m_reflex_muscle.at(FR2_m) < MINFTIOUT)
            {
             m_reflex_muscle.at(FR2_m) = MINFTIOUT;
            }


      m.at(FR0_m) = m_reflex_muscle.at(FR0_m);
      m.at(FR1_m) = m_reflex_muscle.at(FR1_m);
      m.at(FR2_m) = m_reflex_muscle.at(FR2_m);

      m.at(FL0_m) = m_reflex_muscle.at(FL0_m);
      m.at(FL1_m) = m_reflex_muscle.at(FL1_m);
      m.at(FL2_m) = m_reflex_muscle.at(FL2_m);
      


    }



    
    
//    m.at(TR0_m) = 0.4;//m_reflex.at(TR0_m);//*1.2-0.3;
//      m.at(TR1_m) = 0.4;//m_reflex.at(TR1_m);//*1.2+0.2;
//      m.at(TR2_m) = 0.4;//m_reflex.at(TR2_m);//*1.2+0.5;
//
//      m.at(TL0_m) = 0.4;//m_reflex.at(TL0_m);//*1.2-0.3;
//      m.at(TL1_m) = 0.4;//m_reflex.at(TL1_m);//*1.2+0.2;
//      m.at(TL2_m) = 0.4;//m_reflex.at(TL2_m);//*1.2+0.5;
//
//      m.at(CL0_m) = 1.0;//m_reflex.at(CL0_m);
//      m.at(CL1_m) = 1.0;//m_reflex.at(CL1_m);
//      m.at(CL2_m) = 1.0;//m_reflex.at(CL2_m);
//
//      m.at(CR0_m) = 1.0;//m_reflex.at(CR0_m);
//      m.at(CR1_m) = 1.0;//m_reflex.at(CR1_m);
//      m.at(CR2_m) = 1.0;//m_reflex.at(CR2_m);
//
//
//      m.at(FR0_m) = 1.0;//m_reflex.at(FR0_m);
//      m.at(FR1_m) = 1.0;//m_reflex.at(FR1_m);
//      m.at(FR2_m) = 1.0;//m_reflex.at(FR2_m);
//
//      m.at(FL0_m) = 1.0;//m_reflex.at(FL0_m);
//      m.at(FL1_m) = m_reflex.at(FL1_m);
//      m.at(FL2_m) = m_reflex.at(FL2_m);

    m.at(BJ_m) =  m_reflex.at(BJ_m);
    
    
    


  }


  int data = 1;

  //		m.at(TR0_m) = data;//m_reflex.at(TR0_m);
  //		m.at(TR1_m) = data;//m_reflex.at(TR1_m);
  //		m.at(TR2_m) = data;//m_reflex.at(TR2_m);
  //
  //		m.at(TL0_m) = data;//m_reflex.at(TL0_m);
  //		m.at(TL1_m) = data;//m_reflex.at(TL1_m);
  //		m.at(TL2_m) = data;//m_reflex.at(TL2_m);
  //
  //		m.at(CL0_m) = data;//m_reflex.at(CL0_m);
  //		m.at(CL1_m) = data;//m_reflex.at(CL1_m);
  //		m.at(CL2_m) = data;//m_reflex.at(CL2_m);
  //
  //		m.at(CR0_m) = data;//m_reflex.at(CR0_m);
  //		m.at(CR1_m) = data;//m_reflex.at(CR1_m);
  //		m.at(CR2_m) = data;//m_reflex.at(CR2_m);


  //		m.at(FR0_m) = -1*data;//m_reflex.at(FR0_m);
  //		m.at(FR1_m) = -1*data;//m_reflex.at(FR1_m);
  //		m.at(FR2_m) = -1*data;//m_reflex.at(FR2_m);
  //
  //		m.at(FL0_m) = -1*data;//m_reflex.at(FL0_m);
  //		m.at(FL1_m) = -1*data;//m_reflex.at(FL1_m);
  //		m.at(FL2_m) = -1*data;//m_reflex.at(FL2_m);
  //
  //		m.at(BJ_m) =  m_reflex.at(BJ_m);//(m_reflex.at(TR0_m)-0.35)*5;//



  //Converting into deg to check:
  //1) Check MAX and MIN angle in amosII.cpp >>  //Similar to real robot
  //2) take that angles to here MAX, MIN parameters

  m_deg_test.at(TR0_m) = (((m.at(TR0_m)+1.0)/(2.0))*(45 /*MAX*/-(-45 /*MIN*/)))+(-45/*MIN*/);
  m_deg_test.at(TR1_m) = (((m.at(TR1_m)+1.0)/(2.0))*(45-(-45)))+(-45);
  m_deg_test.at(TR2_m) = (((m.at(TR2_m)+1.0)/(2.0))*(45-(-45)))+(-45);
  m_deg_test.at(TL0_m) = (((m.at(TL0_m)+1.0)/(2.0))*(45-(-45)))+(-45);
  m_deg_test.at(TL1_m) = (((m.at(TL1_m)+1.0)/(2.0))*(45-(-45)))+(-45);
  m_deg_test.at(TL2_m) = (((m.at(TL2_m)+1.0)/(2.0))*(45-(-45)))+(-45);

  m_deg_test.at(CR0_m) = (((m.at(CR0_m)+1.0)/(2.0))*(100.0-(-30.0)))+(-30.0);
  m_deg_test.at(CR1_m) = (((m.at(CR1_m)+1.0)/(2.0))*(100.0-(-30.0)))+(-30.0);
  m_deg_test.at(CR2_m) = (((m.at(CR2_m)+1.0)/(2.0))*(100.0-(-30.0)))+(-30.0);
  m_deg_test.at(CL0_m) = (((m.at(CL0_m)+1.0)/(2.0))*(100.0-(-30.0)))+(-30.0);
  m_deg_test.at(CL1_m) = (((m.at(CL1_m)+1.0)/(2.0))*(100.0-(-30.0)))+(-30.0);
  m_deg_test.at(CL2_m) = (((m.at(CL2_m)+1.0)/(2.0))*(100.0-(-30.0)))+(-30.0);

  m_deg_test.at(FR0_m) = (((m.at(FR0_m)+1.0)/(2.0))*(15-(140)))+(140);
  m_deg_test.at(FR1_m) = (((m.at(FR1_m)+1.0)/(2.0))*(15-(140)))+(140);
  m_deg_test.at(FR2_m) = (((m.at(FR2_m)+1.0)/(2.0))*(15-(140)))+(140);
  m_deg_test.at(FL0_m) = (((m.at(FL0_m)+1.0)/(2.0))*(15-(140)))+(140);
  m_deg_test.at(FL1_m) = (((m.at(FL1_m)+1.0)/(2.0))*(15-(140)))+(140);
  m_deg_test.at(FL2_m) = (((m.at(FL2_m)+1.0)/(2.0))*(15-(140)))+(140);
  
  

  
  

  outFilenlc1<<m_reflex.at(CR0_m)<<' '<<m_reflex.at(FR0_m) <<' '<<m_reflex.at(TR0_m)<<' '<<m_reflex.at(CR1_m)<<' '<<m_reflex.at(FR1_m) <<' '<<m_reflex.at(TR1_m)<<' '<<m_reflex.at(CR2_m)<<' '<<m_reflex.at(FR2_m) <<' '<<m_reflex.at(TR2_m)<<endl;

  //std::cout<<m.at(TR0_m)<<" "<<m.at(TR1_m)<<" "<<m.at(TR2_m)<< "\n";



  global_count++;
  //std::cout<<"COUNTER"<<global_count<<"allfoot_off_ground="<<allfoot_off_ground<< "\n";

  if(switchon_purefootsignal)
  {
    std::cout<<"Error = ONLY foot signal"<< "\n";
  }

  return m;

};



