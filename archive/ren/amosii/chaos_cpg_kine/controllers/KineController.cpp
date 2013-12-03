/*
 * KineController.cpp
 *
 *  Created on: Oct 24, 2011
 *      Author: Ren Guanjiao
 */

#include "KineController.h"

//This function is to initialize parameters
KineController::KineController() {
  l1 = 35;
  l2 = 60;
  l3 = 115;
  
  out.resize(19);
  
  /*RF_ini.FTi = -90*M_PI/180; 	//FTi joint
   RF_ini.CTr = 0*M_PI/180;		//CTr joint
   RF_ini.TC = M_PI/6;			//TC joint

   RM_ini.FTi = -90*M_PI/180; 	//FTi joint
   RM_ini.CTr = 0*M_PI/180;		//CTr joint
   RM_ini.TC = 0;				//TC joint

   RH_ini.FTi = -90*M_PI/180; 	//FTi joint
   RH_ini.CTr = 0*M_PI/180;		//CTr joint
   RH_ini.TC = -M_PI/6;			//TC joint

   LF_ini.FTi = 90*M_PI/180; 	//FTi joint
   LF_ini.CTr = -0*M_PI/180;	//CTr joint
   LF_ini.TC = -M_PI/6;			//TC joint

   LM_ini.FTi = 90*M_PI/180; 	//FTi joint
   LM_ini.CTr = -0*M_PI/180;	//CTr joint
   LM_ini.TC = 0;				//TC joint

   LH_ini.FTi = 90*M_PI/180; 	//FTi joint
   LH_ini.CTr = -0*M_PI/180;	//CTr joint
   LH_ini.TC = M_PI/6;			//TC joint
   */
  RF_ini.FTi = -120 * M_PI / 180; //FTi joint
  RF_ini.CTr = 30 * M_PI / 180; //CTr joint
  RF_ini.TC = M_PI / 6; //TC joint
      
  RM_ini.FTi = -120 * M_PI / 180; //FTi joint
  RM_ini.CTr = 30 * M_PI / 180; //CTr joint
  RM_ini.TC = 0; //TC joint
      
  RH_ini.FTi = -120 * M_PI / 180; //FTi joint
  RH_ini.CTr = 30 * M_PI / 180; //CTr joint
  RH_ini.TC = -M_PI / 6; //TC joint
      
  LF_ini.FTi = 120 * M_PI / 180; //FTi joint
  LF_ini.CTr = -30 * M_PI / 180; //CTr joint
  LF_ini.TC = -M_PI / 6; //TC joint
      
  LM_ini.FTi = 120 * M_PI / 180; //FTi joint
  LM_ini.CTr = -30 * M_PI / 180; //CTr joint
  LM_ini.TC = 0; //TC joint
      
  LH_ini.FTi = 120 * M_PI / 180; //FTi joint
  LH_ini.CTr = -30 * M_PI / 180; //CTr joint
  LH_ini.TC = M_PI / 6; //TC joint
      
  legend_RF.Y = legend_RH.Y = legend_LM.Y = -STRIDE / 2;
  legend_LF.Y = legend_LH.Y = legend_RM.Y = STRIDE / 2;
  legend_RF.Z = legend_RH.Z = legend_LM.Z = 0;
  legend_LF.Z = legend_LH.Z = legend_RM.Z = 0;
  legend_RF.X = legend_RH.X = legend_LM.X = 0;
  legend_LF.X = legend_LH.X = legend_RM.X = 0;
  
  swingflag = true;
  step = 0;
  
  //desired point of the RF legs
  desire.X = -20;
  desire.Y = 120;
  desire.Z = 60;
}

KineController::~KineController() {
  ;
}

void KineController::Kine(double tmpRFx, double tmpRFy, double tmpRFz, double tmpRMx, double tmpRMy, double tmpRMz,
    double tmpRHx, double tmpRHy, double tmpRHz, double tmpLFx, double tmpLFy, double tmpLFz, double tmpLMx,
    double tmpLMy, double tmpLMz, double tmpLHx, double tmpLHy, double tmpLHz) {
//	 //-----------------------------move forward-------------------------------------
//   if(swingflag)
//   {
//     legend_RF.Y = legend_RH.Y = legend_LM.Y = ((double)n++/MAXNUM) * STRIDE;
//     legend_RF.Z = legend_RH.Z = legend_LM.Z = - (double) abs(n) * 2 * LEGH / MAXNUM + LEGH;
//     legend_LF.Y = legend_LH.Y = legend_RM.Y = -((double)n/MAXNUM) * STRIDE;
//     legend_LF.Z = legend_LH.Z = legend_RM.Z = 0;
//     if((MAXNUM/2) == n)	 swingflag = false;
//   }
//   else
//   {
//     legend_RF.Y = legend_RH.Y = legend_LM.Y = ((double)n--/MAXNUM) * STRIDE;
//     legend_RF.Z = legend_RH.Z = legend_LM.Z = 0;
//     legend_LF.Y = legend_LH.Y = legend_RM.Y = -((double)n/MAXNUM) * STRIDE;
//     legend_LF.Z = legend_LH.Z = legend_RM.Z = - (double) abs(n) * 2 * LEGH / MAXNUM + LEGH;
//     if((-MAXNUM/2) == n)	 swingflag = true;
//   }
//	 //-----------------------------move forward finish, stop-------------------------------------
  
  legend_RF.X = tmpRFx;
  legend_RF.Y = tmpRFy * STRIDE;
  legend_RF.Z = tmpRFz * LEGH;
  legend_RM.X = tmpRMx;
  legend_RM.Y = tmpRMy * STRIDE;
  legend_RM.Z = tmpRMz * LEGH;
  legend_RH.X = tmpRHx;
  legend_RH.Y = tmpRHy * STRIDE;
  legend_RH.Z = tmpRHz * LEGH;
  legend_LF.X = tmpLFx;
  legend_LF.Y = tmpLFy * STRIDE;
  legend_LF.Z = tmpLFz * LEGH;
  legend_LM.X = tmpLMx;
  legend_LM.Y = tmpLMy * STRIDE;
  legend_LM.Z = tmpLMz * LEGH;
  legend_LH.X = tmpLHx;
  legend_LH.Y = tmpLHy * STRIDE;
  legend_LH.Z = tmpLHz * LEGH;
  
  //--------------control the six legs-------------------------
  SingleLegInverKine_RF(legend_RF.X, legend_RF.Y, legend_RF.Z);
  SingleLegInverKine_RM(legend_RM.X, legend_RM.Y, legend_RM.Z);
  SingleLegInverKine_RH(legend_RH.X, legend_RH.Y, legend_RH.Z);
  SingleLegInverKine_LF(legend_LF.X, legend_LF.Y, legend_LF.Z);
  SingleLegInverKine_LM(legend_LM.X, legend_LM.Y, legend_LM.Z);
  SingleLegInverKine_LH(legend_LH.X, legend_LH.Y, legend_LH.Z);
  
  //--------------------mapping to -1 ~ 1 -----------------------
  out.at(TR0_m) = RF.TC * 0.0143 * 180 / M_PI;
  out.at(TR1_m) = RM.TC * 0.0167 * 180 / M_PI;
  out.at(TR2_m) = RH.TC * 0.0143 * 180 / M_PI;
  
  out.at(TL0_m) = -LF.TC * 0.0143 * 180 / M_PI;
  out.at(TL1_m) = -LM.TC * 0.0167 * 180 / M_PI;
  out.at(TL2_m) = -LH.TC * 0.0143 * 180 / M_PI;
  
  out.at(CL0_m) = -LF.CTr * 0.0143 * 180 / M_PI;
  out.at(CL1_m) = -LM.CTr * 0.0143 * 180 / M_PI;
  out.at(CL2_m) = -LH.CTr * 0.0143 * 180 / M_PI;
  
  out.at(CR0_m) = RF.CTr * 0.0143 * 180 / M_PI;
  out.at(CR1_m) = RM.CTr * 0.0143 * 180 / M_PI;
  out.at(CR2_m) = RH.CTr * 0.0143 * 180 / M_PI;
  
  out.at(FR0_m) = RF.FTi * 0.0182 * 180 / M_PI + 1.3636;
  out.at(FR1_m) = RM.FTi * 0.0182 * 180 / M_PI + 1.3636;
  out.at(FR2_m) = RH.FTi * 0.0182 * 180 / M_PI + 1.3636;
  
  out.at(FL0_m) = -LF.FTi * 0.0182 * 180 / M_PI + 1.3636;
  out.at(FL1_m) = -LM.FTi * 0.0182 * 180 / M_PI + 1.3636;
  out.at(FL2_m) = -LH.FTi * 0.0182 * 180 / M_PI + 1.3636;
  
}

void KineController::SingleLegInverKine_RF(double x, double y, double z) {
  double L;
  Point pA_b;
  Point pD_b;
  
  pA_b.X = (l1 + l2 * cos(RF_ini.CTr) + l3 * cos(-RF_ini.FTi - RF_ini.CTr)) * cos(RF_ini.TC) + x; //X coordinate
  pA_b.Y = (l1 + l2 * cos(RF_ini.CTr) + l3 * cos(-RF_ini.FTi - RF_ini.CTr)) * sin(RF_ini.TC) + y; //Y coordinate
  pA_b.Z = l2 * sin(RF_ini.CTr) - l3 * sin(-RF_ini.FTi - RF_ini.CTr) + z; //Z coordinate
      
  RF.TC = atan(pA_b.Y / pA_b.X); //angle of TC- joint
      
  pD_b.X = l1 * cos(RF.TC);
  pD_b.Y = l1 * sin(RF.TC);
  pD_b.Z = 0;
  
  //H = l3 * sin( -RF_ini.FTi - RF_ini.CTr ) - l2 * sin( RF_ini.CTr ); //height of body
  
  L = sqrt(
      (pA_b.X - pD_b.X) * (pA_b.X - pD_b.X) + (pA_b.Y - pD_b.Y) * (pA_b.Y - pD_b.Y)
          + (pA_b.Z - pD_b.Z) * (pA_b.Z - pD_b.Z));
  
  RF.FTi = acos((l2 * l2 + l3 * l3 - L * L) / (2 * l2 * l3)) - M_PI; //angle of FTi- joint
  
  RF.CTr = acos((L * L + l2 * l2 - l3 * l3) / (2 * L * l2)) + asin((pA_b.Z - pD_b.Z) / L); //angle of CTr- joint
      
  //std::cout<<RF.FTi*180/M_PI<<"  "<<RF.CTr*180/M_PI<<"  "<<RF.TC*180/M_PI<<endl;
  //std::cout<<"X == "<<pA_b.X<<"  Y="<<pA_b.Y<<"  Z="<<pA_b.Z<<endl;
}

void KineController::SingleLegInverKine_RM(double x, double y, double z) {
  double L;
  Point pA_b;
  Point pD_b;
  
  pA_b.X = (l1 + l2 * cos(RM_ini.CTr) + l3 * cos(-RM_ini.FTi - RM_ini.CTr)) * cos(RM_ini.TC) + x; //X coordinate
  pA_b.Y = (l1 + l2 * cos(RM_ini.CTr) + l3 * cos(-RM_ini.FTi - RM_ini.CTr)) * sin(RM_ini.TC) + y; //Y coordinate
  pA_b.Z = l2 * sin(RM_ini.CTr) - l3 * sin(-RM_ini.FTi - RM_ini.CTr) + z; //Z coordinate
      
  RM.TC = atan(pA_b.Y / pA_b.X); //angle of TC- joint
      
  pD_b.X = l1 * cos(RM.TC);
  pD_b.Y = l1 * sin(RM.TC);
  pD_b.Z = 0;
  
  //H = l3 * sin( -RF_ini.FTi - RF_ini.CTr ) - l2 * sin( RF_ini.CTr ); //height of body
  
  L = sqrt(
      (pA_b.X - pD_b.X) * (pA_b.X - pD_b.X) + (pA_b.Y - pD_b.Y) * (pA_b.Y - pD_b.Y)
          + (pA_b.Z - pD_b.Z) * (pA_b.Z - pD_b.Z));
  
  RM.FTi = acos((l2 * l2 + l3 * l3 - L * L) / (2 * l2 * l3)) - M_PI; //angle of FTi- joint
  
  RM.CTr = acos((L * L + l2 * l2 - l3 * l3) / (2 * L * l2)) + asin((pA_b.Z - pD_b.Z) / L); //angle of CTr- joint
      
  //std::cout<<RM.FTi*180/M_PI<<"  "<<RM.CTr*180/M_PI<<"  "<<RM.TC*180/M_PI<<endl;
  //std::cout<<"X == "<<pA_b.X<<"  Y="<<pA_b.Y<<"  Z="<<pA_b.Z<<endl;
}

void KineController::SingleLegInverKine_RH(double x, double y, double z) {
  double L;
  Point pA_b;
  Point pD_b;
  
  pA_b.X = (l1 + l2 * cos(RH_ini.CTr) + l3 * cos(-RH_ini.FTi - RH_ini.CTr)) * cos(RH_ini.TC) + x; //X coordinate
  pA_b.Y = (l1 + l2 * cos(RH_ini.CTr) + l3 * cos(-RH_ini.FTi - RH_ini.CTr)) * sin(RH_ini.TC) + y; //Y coordinate
  pA_b.Z = l2 * sin(RH_ini.CTr) - l3 * sin(-RH_ini.FTi - RH_ini.CTr) + z; //Z coordinate
      
  RH.TC = atan(pA_b.Y / pA_b.X); //angle of TC- joint
      
  pD_b.X = l1 * cos(RH.TC);
  pD_b.Y = l1 * sin(RH.TC);
  pD_b.Z = 0;
  
  //H = l3 * sin( -RF_ini.FTi - RF_ini.CTr ) - l2 * sin( RF_ini.CTr ); //height of body
  
  L = sqrt(
      (pA_b.X - pD_b.X) * (pA_b.X - pD_b.X) + (pA_b.Y - pD_b.Y) * (pA_b.Y - pD_b.Y)
          + (pA_b.Z - pD_b.Z) * (pA_b.Z - pD_b.Z));
  
  RH.FTi = acos((l2 * l2 + l3 * l3 - L * L) / (2 * l2 * l3)) - M_PI; //angle of FTi- joint
  
  RH.CTr = acos((L * L + l2 * l2 - l3 * l3) / (2 * L * l2)) + asin((pA_b.Z - pD_b.Z) / L); //angle of CTr- joint
      
  //std::cout<<RH.FTi*180/M_PI<<"  "<<RH.CTr*180/M_PI<<"  "<<RH.TC*180/M_PI<<endl;
}

void KineController::SingleLegInverKine_LF(double x, double y, double z) {
  double L;
  Point pA_b;
  Point pD_b;
  
  pA_b.X = (-l1 - l2 * cos(LF_ini.CTr) - l3 * cos(LF_ini.FTi + LF_ini.CTr)) * cos(LF_ini.TC) + x; //X coordinate
  pA_b.Y = (-l1 - l2 * cos(LF_ini.CTr) - l3 * cos(LF_ini.FTi + LF_ini.CTr)) * sin(LF_ini.TC) + y; //Y coordinate
  pA_b.Z = -l2 * sin(LF_ini.CTr) - l3 * sin(LF_ini.FTi + LF_ini.CTr) + z; //Z coordinate
      
  LF.TC = atan(pA_b.Y / pA_b.X); //angle of TC- joint
      
  pD_b.X = -l1 * cos(LF.TC);
  pD_b.Y = -l1 * sin(LF.TC);
  pD_b.Z = 0;
  
  //H = l3 * sin( -RF_ini.FTi - RF_ini.CTr ) - l2 * sin( RF_ini.CTr ); //height of body
  
  L = sqrt(
      (pA_b.X - pD_b.X) * (pA_b.X - pD_b.X) + (pA_b.Y - pD_b.Y) * (pA_b.Y - pD_b.Y)
          + (pA_b.Z - pD_b.Z) * (pA_b.Z - pD_b.Z));
  
  LF.FTi = M_PI - acos((l2 * l2 + l3 * l3 - L * L) / (2 * l2 * l3)); //angle of FTi- joint
      
  LF.CTr = -acos((L * L + l2 * l2 - l3 * l3) / (2 * L * l2)) - asin((pA_b.Z - pD_b.Z) / L); //angle of CTr- joint
      
  //std::cout<<LF.FTi*180/M_PI<<"  "<<LF.CTr*180/M_PI<<"  "<<LF.TC*180/M_PI<<endl;
}

void KineController::SingleLegInverKine_LM(double x, double y, double z) {
  double L;
  Point pA_b;
  Point pD_b;
  
  pA_b.X = (-l1 - l2 * cos(LM_ini.CTr) - l3 * cos(LM_ini.FTi + LM_ini.CTr)) * cos(LM_ini.TC) + x; //X coordinate
  pA_b.Y = (-l1 - l2 * cos(LM_ini.CTr) - l3 * cos(LM_ini.FTi + LM_ini.CTr)) * sin(LM_ini.TC) + y; //Y coordinate
  pA_b.Z = -l2 * sin(LM_ini.CTr) - l3 * sin(LM_ini.FTi + LM_ini.CTr) + z; //Z coordinate
      
  LM.TC = atan(pA_b.Y / pA_b.X); //angle of TC- joint
      
  pD_b.X = -l1 * cos(LM.TC);
  pD_b.Y = -l1 * sin(LM.TC);
  pD_b.Z = 0;
  
  //H = l3 * sin( -RF_ini.FTi - RF_ini.CTr ) - l2 * sin( RF_ini.CTr ); //height of body
  
  L = sqrt(
      (pA_b.X - pD_b.X) * (pA_b.X - pD_b.X) + (pA_b.Y - pD_b.Y) * (pA_b.Y - pD_b.Y)
          + (pA_b.Z - pD_b.Z) * (pA_b.Z - pD_b.Z));
  
  LM.FTi = M_PI - acos((l2 * l2 + l3 * l3 - L * L) / (2 * l2 * l3)); //angle of FTi- joint
      
  LM.CTr = -acos((L * L + l2 * l2 - l3 * l3) / (2 * L * l2)) - asin((pA_b.Z - pD_b.Z) / L); //angle of CTr- joint
      
  //std::cout<<LM.FTi*180/M_PI<<"  "<<LM.CTr*180/M_PI<<"  "<<LM.TC*180/M_PI<<endl;
}

void KineController::SingleLegInverKine_LH(double x, double y, double z) {
  double L;
  Point pA_b;
  Point pD_b;
  
  pA_b.X = (-l1 - l2 * cos(LH_ini.CTr) - l3 * cos(LH_ini.FTi + LH_ini.CTr)) * cos(LH_ini.TC) + x; //X coordinate
  pA_b.Y = (-l1 - l2 * cos(LH_ini.CTr) - l3 * cos(LH_ini.FTi + LH_ini.CTr)) * sin(LH_ini.TC) + y; //Y coordinate
  pA_b.Z = -l2 * sin(LH_ini.CTr) - l3 * sin(LH_ini.FTi + LH_ini.CTr) + z; //Z coordinate
      
  LH.TC = atan(pA_b.Y / pA_b.X); //angle of TC- joint
      
  pD_b.X = -l1 * cos(LH.TC);
  pD_b.Y = -l1 * sin(LH.TC);
  pD_b.Z = 0;
  
  //H = l3 * sin( -RF_ini.FTi - RF_ini.CTr ) - l2 * sin( RF_ini.CTr ); //height of body
  
  L = sqrt(
      (pA_b.X - pD_b.X) * (pA_b.X - pD_b.X) + (pA_b.Y - pD_b.Y) * (pA_b.Y - pD_b.Y)
          + (pA_b.Z - pD_b.Z) * (pA_b.Z - pD_b.Z));
  
  LH.FTi = M_PI - acos((l2 * l2 + l3 * l3 - L * L) / (2 * l2 * l3)); //angle of FTi- joint
      
  LH.CTr = -acos((L * L + l2 * l2 - l3 * l3) / (2 * L * l2)) - asin((pA_b.Z - pD_b.Z) / L); //angle of CTr- joint
      
  //std::cout<<LH.FTi*180/M_PI<<"  "<<LH.CTr*180/M_PI<<"  "<<LH.TC*180/M_PI<<endl;
}

