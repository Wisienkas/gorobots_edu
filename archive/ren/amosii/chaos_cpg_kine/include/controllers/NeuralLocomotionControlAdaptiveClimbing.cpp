/*
 * NeuralLocomotionControlAdaptiveClimbing.cpp
 *
 *  Created on: May 2, 2011
 *      Author: poramate
 */

#include "NeuralLocomotionControlAdaptiveClimbing.h"

//initialize object by designed Ren Guanjiao
#include "ChaoscontrAndPostproc.h"
ChaoscontrAndPostproc ChaosCont(4);

//3) Step function of Neural locomotion control------

NeuralLocomotionControlAdaptiveClimbing::NeuralLocomotionControlAdaptiveClimbing() {
  
  //Save files
  outFilenlc1.open("FootContactTraj.txt");
  
  /*******************************************************************************
   * Set vector size
   *******************************************************************************/
  //---Set vector size----//
  //Input vector size
  input.resize(5);
  fs.resize(6);
  
  //CPG vector sizes
  cpg_activity.resize(2);
  cpg_output.resize(2);
  
  cpg_w.resize(2);
  for (unsigned int i = 0; i < cpg_w.size(); i++) {
    cpg_w.at(i).resize(2);
  }
  
  //Delay line
  buffer_cpg.resize(126);
  cpg_leg.resize(6);
  cpg_pre.resize(6);
  
  //Trajectory generator
  Tj.resize(6);
  
  //Force Feedback
  Tj_ForceFB.resize(6);
  Tj_pre.resize(6);
  
  //motor neuron
  m.resize(19);
  
  /*******************************************************************************
   *  Initial parameters
   *******************************************************************************/
  //---Initial parameters----//
  //---Inputs
  input.at(0) = 1;
  input.at(1) = 1;
  input.at(2) = 1; // 0, or 1
  input.at(3) = 1;
  input.at(4) = 1;
  
  //motor time delay
  tau = 18; //16;
  tau_l = 90; //48;
      
  //global_count
  global_count = 0;
  
}
;

NeuralLocomotionControlAdaptiveClimbing::~NeuralLocomotionControlAdaptiveClimbing() {
  
  //Save files
  outFilenlc1.close();
  
}
;

std::vector<double> NeuralLocomotionControlAdaptiveClimbing::step_nlc(const std::vector<double> in0,
    const std::vector<double> in1) { //, bool Footinhibition){
    
  //Input to control locomotion
  input.at(0) = 0; //walking = 0, joint inhibition = 1
  input.at(1) = 1; //lateral = 0, no lateral motion = 1
  input.at(2) = 1; //lateral right = 0, lateral left = 1
  input.at(3) = -1; //turn left = 1,
  input.at(4) = -1; //turn right = 1,
      
  fs.at(R0) = in0.at(R0_fs);
  fs.at(R1) = in0.at(R1_fs);
  fs.at(R2) = in0.at(R2_fs);
  fs.at(L0) = in0.at(L0_fs);
  fs.at(L1) = in0.at(L1_fs);
  fs.at(L2) = in0.at(L2_fs);
  
  /*******************************************************************************
   *  MODULE 1 CPG
   *******************************************************************************/

  //---ChaosCPG + post processing---//
  //starting!
  if (global_count == 0) {
    ChaosCont.setPeriod(4);
  }
  
  //ChaosControl CPG
  ChaosCont.ChaosControl();
  cpg_output.at(0) = ChaosCont.getOutput1();
  cpg_output.at(1) = ChaosCont.getOutput2();
  
  //---End ChaosCPG + post processing---//
  
  /*******************************************************************************
   *  MODULE 2 Delay line
   *******************************************************************************/
  buffer_cpg.push_back(cpg_output.at(0));

  //remember the value in last run
  for (int i = 0; i < 6; i++) {
    cpg_pre.at(i) = cpg_leg.at(i);
  }
  
  //R0(RF) leg ---------------------------------
  if ((buffer_cpg.size() - 2 * tau) > 0) {
    cpg_leg.at(R0) = buffer_cpg[buffer_cpg.size() - 2 * tau - 1];
  } else {
    cpg_leg.at(R0) = 0;
  }
  
  //R1(RM) leg ---------------------------------
  if ((buffer_cpg.size() - tau) > 0) {
    cpg_leg.at(R1) = buffer_cpg[buffer_cpg.size() - tau - 1];
  } else {
    cpg_leg.at(R1) = 0;
  }
  
  //R2(RH) leg ---------------------------------
  if (buffer_cpg.size() > 0) {
    cpg_leg.at(R2) = buffer_cpg[buffer_cpg.size() - 1];
  } else {
    cpg_leg.at(R2) = 0;
  }
  
  //L0(LF) leg ---------------------------------
  if ((buffer_cpg.size() - 2 * tau - tau_l) > 0) {
    cpg_leg.at(L0) = buffer_cpg[buffer_cpg.size() - 2 * tau - tau_l - 1];
  } else {
    cpg_leg.at(L0) = 0;
  }
  
  //L1(LM) leg ---------------------------------
  if ((buffer_cpg.size() - tau - tau_l) > 0) {
    cpg_leg.at(L1) = buffer_cpg[buffer_cpg.size() - tau - tau_l - 1];
  } else {
    cpg_leg.at(L1) = 0;
  }
  
  //L2(LH) leg ---------------------------------
  if ((buffer_cpg.size() - tau_l) > 0) {
    cpg_leg.at(L2) = buffer_cpg[buffer_cpg.size() - tau_l - 1];
  } else {
    cpg_leg.at(L2) = 0;
  }
  
    //------------------output file--------------------------//
    outFilenlc1 <<cpg_leg.at(R0)<<"  "
                <<cpg_leg.at(R1)<<"  "
                <<cpg_leg.at(R2)<<"  "
                <<cpg_leg.at(L0)<<"  "
                <<cpg_leg.at(L1)<<"  "
                <<cpg_leg.at(L2)<<endl;
    cout<<"buffer_cpg.size()="<<buffer_cpg.size()<<endl;
    //--------------end output file--------------------------//

  /*******************************************************************************
   *  MODULE 3 Trajectory Generator
   *******************************************************************************/
  //R0(RF) leg ---------------------------------
  Tj.at(R0).x = 0;
  Tj.at(R0).y = cpg_leg.at(R0);
  if ((cpg_leg.at(R0) > cpg_pre.at(R0)) && (cpg_leg.at(R0) < 0)) {
    Tj.at(R0).z = cpg_leg.at(R0) + 1;
    leg_state[R0] = SWI_EL; //This leg is in swing_elavation state
  } else if ((cpg_leg.at(R0) > cpg_pre.at(R0)) && (cpg_leg.at(R0) >= 0)) {
    Tj.at(R0).z = -cpg_leg.at(R0) + 1;
    leg_state[R0] = SWI_DP; //This leg is in swing_depress state
  } else {
    Tj.at(R0).z = 0;
    leg_state[R0] = SUP; //This leg is in support phase
  }
  
  //R1(RM) leg ---------------------------------
  Tj.at(R1).x = 0;
  Tj.at(R1).y = cpg_leg.at(R1);
  if ((cpg_leg.at(R1) > cpg_pre.at(R1)) && (cpg_leg.at(R1) < 0)) {
    Tj.at(R1).z = cpg_leg.at(R1) + 1;
    leg_state[R1] = SWI_EL; //This leg is in swing_elavation state
  } else if ((cpg_leg.at(R1) > cpg_pre.at(R1)) && (cpg_leg.at(R1) >= 0)) {
    Tj.at(R1).z = -cpg_leg.at(R1) + 1;
    leg_state[R1] = SWI_DP; //This leg is in swing_depress state
  } else {
    Tj.at(R1).z = 0;
    leg_state[R1] = SUP; //This leg is in support phase
  }
  
  //R2(RH) leg ---------------------------------
  Tj.at(R2).x = 0;
  Tj.at(R2).y = cpg_leg.at(R2);
  if ((cpg_leg.at(R2) > cpg_pre.at(R2)) && (cpg_leg.at(R2) < 0)) {
    Tj.at(R2).z = cpg_leg.at(R2) + 1;
    leg_state[R2] = SWI_EL; //This leg is in swing_elavation state
  } else if ((cpg_leg.at(R2) > cpg_pre.at(R2)) && (cpg_leg.at(R2) >= 0)) {
    Tj.at(R2).z = -cpg_leg.at(R2) + 1;
    leg_state[R2] = SWI_DP; //This leg is in swing_depress state
  } else {
    Tj.at(R2).z = 0;
    leg_state[R2] = SUP; //This leg is in support phase
  }
  
  //L0(LF) leg ---------------------------------
  Tj.at(L0).x = 0;
  Tj.at(L0).y = cpg_leg.at(L0);
  if ((cpg_leg.at(L0) > cpg_pre.at(L0)) && (cpg_leg.at(L0) < 0)) {
    Tj.at(L0).z = cpg_leg.at(L0) + 1;
    leg_state[L0] = SWI_EL; //This leg is in swing_elavation state
  } else if ((cpg_leg.at(L0) > cpg_pre.at(L0)) && (cpg_leg.at(L0) >= 0)) {
    Tj.at(L0).z = -cpg_leg.at(L0) + 1;
    leg_state[L0] = SWI_DP; //This leg is in swing_depress state
  } else {
    Tj.at(L0).z = 0;
    leg_state[L0] = SUP; //This leg is in support phase
  }
  
  //L1(LM) leg ---------------------------------
  Tj.at(L1).x = 0;
  Tj.at(L1).y = cpg_leg.at(L1);
  if ((cpg_leg.at(L1) > cpg_pre.at(L1)) && (cpg_leg.at(L1) < 0)) {
    Tj.at(L1).z = cpg_leg.at(L1) + 1;
    leg_state[L1] = SWI_EL; //This leg is in swing_elavation state
  } else if ((cpg_leg.at(L1) > cpg_pre.at(L1)) && (cpg_leg.at(L1) >= 0)) {
    Tj.at(L1).z = -cpg_leg.at(L1) + 1;
    leg_state[L1] = SWI_DP; //This leg is in swing_depress state
  } else {
    Tj.at(L1).z = 0;
    leg_state[L1] = SUP; //This leg is in support phase
  }
  
  //L2(LH) leg ---------------------------------
  Tj.at(L2).x = 0;
  Tj.at(L2).y = cpg_leg.at(L2);
  if ((cpg_leg.at(L2) > cpg_pre.at(L2)) && (cpg_leg.at(L2) < 0)) {
    Tj.at(L2).z = cpg_leg.at(L2) + 1;
    leg_state[L2] = SWI_EL; //This leg is in swing_elavation state
  } else if ((cpg_leg.at(L2) > cpg_pre.at(L2)) && (cpg_leg.at(L2) >= 0)) {
    Tj.at(L2).z = -cpg_leg.at(L2) + 1;
    leg_state[L2] = SWI_DP; //This leg is in swing_depress state
  } else {
    Tj.at(L2).z = 0;
    leg_state[L2] = SUP; //This leg is in support phase
  }
  
  /*******************************************************************************
   *  MODULE 4 Force Feedback
   *******************************************************************************/
  for (int i = 0; i < 6; i++) {
    Tj_ForceFB.at(i) = Tj.at(i);

    if ( (SWI_DP == leg_state[i]) || (SUP == leg_state[i]) ) {

      if (fs.at(i) < F_L) { //S1: The leg didn't reach the desired position and force is less than FL, keep depressing
        Tj_ForceFB.at(i).z = Tj_pre.at(i).z - 0.06;
      }
      if ( (fs.at(i) >= F_L) && (fs.at(i) <= F_H) && Tj_pre.at(i).z <= 0.0 ) {
        //S2: F_L <= fs.at(i) <= F_H and d >= Dpre, maintain depression
        Tj_ForceFB.at(i).z = Tj_pre.at(i).z;
      }
      if ( (fs.at(i) > F_H) && Tj_pre.at(i).z <= 0.0 ) {
        //S3: fs.at(i) > F_H and d >= Dpre, slow elevate
        Tj_ForceFB.at(i).z = Tj_pre.at(i).z + 0.02;
      }
      if ( (fs.at(i) >= F_L) && Tj_pre.at(i).z > 0.0 ) {
        //S4: fs.at(i) >= F_L and d < Dpre, slow depress
        Tj_ForceFB.at(i).z = Tj_pre.at(i).z - 0.02;
      }

      //position limitation
      if (Tj_ForceFB.at(i).z < -0.5) { //lowest position limitation
        Tj_ForceFB.at(i).z = -0.5;
      } else if ( Tj_ForceFB.at(i).z > 0.5 ) { // highest positioin limitation
        Tj_ForceFB.at(i).z = 0.5;
      }


//    } else if (SWI_EL == leg_state[i]) {
//      Tj_ForceFB.at(i).z = Tj_pre.at(i).z + 0.25;
    }

    Tj_pre.at(i) = Tj_ForceFB.at(i); //record the trajectory for next step
  }
  
//  //------------------output file--------------------------//
//  outFilenlc1 <<Tj_ForceFB.at(R0).z<<"  "
//              <<Tj_ForceFB.at(R1).z<<"  "
//              <<Tj_ForceFB.at(R2).z<<"  "
//              <<Tj_ForceFB.at(L0).z<<"  "
//              <<Tj_ForceFB.at(L1).z<<"  "
//              <<Tj_ForceFB.at(L2).z<<endl;
//  //--------------end output file--------------------------//

  /*******************************************************************************
   *  MODULE 5 Inverse Kinematics
   *******************************************************************************/
  robot.Kine(Tj_ForceFB.at(R0).x, Tj_ForceFB.at(R0).y, Tj_ForceFB.at(R0).z, Tj_ForceFB.at(R1).x, Tj_ForceFB.at(R1).y,
      Tj_ForceFB.at(R1).z, Tj_ForceFB.at(R2).x, Tj_ForceFB.at(R2).y, Tj_ForceFB.at(R2).z, Tj_ForceFB.at(L0).x,
      Tj_ForceFB.at(L0).y, Tj_ForceFB.at(L0).z, Tj_ForceFB.at(L1).x, Tj_ForceFB.at(L1).y, Tj_ForceFB.at(L1).z,
      Tj_ForceFB.at(L2).x, Tj_ForceFB.at(L2).y, Tj_ForceFB.at(L2).z);
  
  /*******************************************************************************
   *  MODULE 6 Motor Control
   *******************************************************************************/

  //---Motor output---//
  m.at(TR0_m) = robot.out.at(TR0_m);
  m.at(TR1_m) = robot.out.at(TR1_m);
  m.at(TR2_m) = robot.out.at(TR2_m);
  
  m.at(TL0_m) = robot.out.at(TL0_m);
  m.at(TL1_m) = robot.out.at(TL1_m);
  m.at(TL2_m) = robot.out.at(TL2_m);
  
  m.at(CL0_m) = robot.out.at(CL0_m);
  m.at(CL1_m) = robot.out.at(CL1_m);
  m.at(CL2_m) = robot.out.at(CL2_m);
  
  m.at(CR0_m) = robot.out.at(CR0_m);
  m.at(CR1_m) = robot.out.at(CR1_m);
  m.at(CR2_m) = robot.out.at(CR2_m);
  
  m.at(FR0_m) = robot.out.at(FR0_m);
  m.at(FR1_m) = robot.out.at(FR1_m);
  m.at(FR2_m) = robot.out.at(FR2_m);
  
  m.at(FL0_m) = robot.out.at(FL0_m);
  m.at(FL1_m) = robot.out.at(FL1_m);
  m.at(FL2_m) = robot.out.at(FL2_m);
  
  m.at(BJ_m) = 0; //(m_reflex.at(TR0_m)-0.35)*5;//
  //---End Motor output---//
  
  global_count++;
  
  return m;
  
}
;

