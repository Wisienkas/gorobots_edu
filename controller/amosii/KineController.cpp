/*
 * KineController.cpp
 *
 *  Created on: Oct 24, 2011
 *      Author: Ren Guanjiao
 */

#include "KineController.h"

//This function is to initialize parameters
KineController::KineController(int amos_version)
{
  if (amos_version==1){
    /* specific parameters for amosii version 1 */

    l1 = 35;
    l2 = 60;
    l3 = 115;

    out.resize(19);

    RF_ini.FTi = -120*PI/180; 	//FTi joint
    RF_ini.CTr = 40*PI/180;//30*PI/180;		//CTr joint
    RF_ini.TC = PI/6;			//TC joint

    RM_ini.FTi = -120*PI/180; 	//FTi joint
    RM_ini.CTr = 40*PI/180;//30*PI/180;		//CTr joint
    RM_ini.TC = 0;				//TC joint

    RH_ini.FTi = -120*PI/180; 	//FTi joint
    RH_ini.CTr = 40*PI/180;//30*PI/180;		//CTr joint
    RH_ini.TC = -PI/6;			//TC joint

    LF_ini.FTi = 120*PI/180; 	//FTi joint
    LF_ini.CTr = -40*PI/180;	//CTr joint
    LF_ini.TC = -PI/6;			//TC joint

    LM_ini.FTi = 120*PI/180; 	//FTi joint
    LM_ini.CTr = -40*PI/180;//-30*PI/180;	//CTr joint
    LM_ini.TC = 0;				//TC joint

    LH_ini.FTi = 120*PI/180; 	//FTi joint
    LH_ini.CTr = -40*PI/180;//-30*PI/180;	//CTr joint
    LH_ini.TC = PI/6;			//TC joint


    n = -MAXNUM/2;
    legend_RF.Y = legend_RH.Y = legend_LM.Y = -STRIDE/2;
    legend_LF.Y = legend_LH.Y = legend_RM.Y = STRIDE/2;
    legend_RF.Z = legend_RH.Z = legend_LM.Z = 0;
    legend_LF.Z = legend_LH.Z = legend_RM.Z = 0;
    legend_RF.X = legend_RH.X = legend_LM.X = 0;
    legend_LF.X = legend_LH.X = legend_RM.X = 0;

    legend_pre_RF.X = legend_pre_RH.X = legend_pre_LM.X = 0;
    legend_pre_LF.X = legend_pre_LH.X = legend_pre_RM.X = 0;
    legend_pre_RF.Y = legend_pre_RH.Y = legend_pre_LM.Y = 0;
    legend_pre_LF.Y = legend_pre_LH.Y = legend_pre_RM.Y = 0;
    legend_pre_RF.Z = legend_pre_RH.Z = legend_pre_LM.Z = 0;
    legend_pre_LF.Z = legend_pre_LH.Z = legend_pre_RM.Z = 0;

    swingflag = true;
    count = 0;
    step = 0;
    step2 = 0;

    desire.X = -10;
    desire.Y = 140;
    desire.Z = 120;


  } else if(amos_version==2){
    /* specific parameters for amosii version 2 */

    l1 = 35;
    l2 = 60;
    l3 = 115;

    out.resize(19);

    RF_ini.FTi = -120*PI/180; 	//FTi joint
    RF_ini.CTr = 30*PI/180;		//CTr joint
    RF_ini.TC = PI/6;			//TC joint

    RM_ini.FTi = -120*PI/180; 	//FTi joint
    RM_ini.CTr = 30*PI/180;		//CTr joint
    RM_ini.TC = 0;				//TC joint

    RH_ini.FTi = -120*PI/180; 	//FTi joint
    RH_ini.CTr = 30*PI/180;		//CTr joint
    RH_ini.TC = -PI/6;			//TC joint

    LF_ini.FTi = 120*PI/180; 	//FTi joint
    LF_ini.CTr = -30*PI/180;	//CTr joint
    LF_ini.TC = -PI/6;			//TC joint

    LM_ini.FTi = 120*PI/180; 	//FTi joint
    LM_ini.CTr = -30*PI/180;	//CTr joint
    LM_ini.TC = 0;				//TC joint

    LH_ini.FTi = 120*PI/180; 	//FTi joint
    LH_ini.CTr = -30*PI/180;	//CTr joint
    LH_ini.TC = PI/6;			//TC joint


    n = -MAXNUM/2;
    legend_RF.Y = legend_RH.Y = legend_LM.Y = -STRIDE/2;
    legend_LF.Y = legend_LH.Y = legend_RM.Y = STRIDE/2;
    legend_RF.Z = legend_RH.Z = legend_LM.Z = 0;
    legend_LF.Z = legend_LH.Z = legend_RM.Z = 0;
    legend_RF.X = legend_RH.X = legend_LM.X = 0;
    legend_LF.X = legend_LH.X = legend_RM.X = 0;

    legend_pre_RF.X = legend_pre_RH.X = legend_pre_LM.X = 0;
    legend_pre_LF.X = legend_pre_LH.X = legend_pre_RM.X = 0;
    legend_pre_RF.Y = legend_pre_RH.Y = legend_pre_LM.Y = 0;
    legend_pre_LF.Y = legend_pre_LH.Y = legend_pre_RM.Y = 0;
    legend_pre_RF.Z = legend_pre_RH.Z = legend_pre_LM.Z = 0;
    legend_pre_LF.Z = legend_pre_LH.Z = legend_pre_RM.Z = 0;

    swingflag = true;
    count = 0;
    step = 0;
    step2 = 0;

    desire.X = -10;
    desire.Y = 140;
    desire.Z = 120;



  } else {std::cout<<"provide correct version of amos!! (1 or 2)"<<std::endl;}

  walking= false;   // for key control of robot
  not_walking = false;
  lift_leg=false;
  lower_leg=false;
  come_back=false;
  not_come_back= false;

  turn_right = false;
  turn_left = false;
  sideward_right = false;
  sideward_left = false;
  turnright_sideward = false;
  turnleft_sideward = false;
  backward_left = false;
  backward_right = false;
}

KineController::~KineController()
{
  ;
}

void KineController::Kine()
{
  //-----------------------------move forward-------------------------------------
  //if (count < 400)

  if (walking)
  {

    //-----------------Forward walking------------------------------------------------------------//
    std::cout<<"forward "<<std::endl;
    if(swingflag)
    {
      legend_RF.Y = legend_RH.Y = legend_LM.Y = ((double)n++/MAXNUM) * STRIDE;
      legend_RF.Z = legend_RH.Z = legend_LM.Z = - (double) abs(n) * 2 * LEGH / MAXNUM + LEGH;
      legend_LF.Y = legend_LH.Y = legend_RM.Y = -((double)n/MAXNUM) * STRIDE;
      legend_LF.Z = legend_LH.Z = legend_RM.Z = 0;
      if((MAXNUM/2) == n)	 swingflag = false;
    }
    else
    {
      legend_RF.Y = legend_RH.Y = legend_LM.Y = ((double)n--/MAXNUM) * STRIDE;
      legend_RF.Z = legend_RH.Z = legend_LM.Z = 0;
      legend_LF.Y = legend_LH.Y = legend_RM.Y = -((double)n/MAXNUM) * STRIDE;
      legend_LF.Z = legend_LH.Z = legend_RM.Z = - (double) abs(n) * 2 * LEGH / MAXNUM + LEGH;
      if((-MAXNUM/2) == n)	 swingflag = true;
    }
    legend_pre_RF = legend_RF;
    legend_pre_RM = legend_RM;
    legend_pre_RH = legend_RH;
    legend_pre_LF = legend_LF;
    legend_pre_LM = legend_LM;
    legend_pre_LH = legend_LH;

    step2 = 0;

  }
  //-----------------------------move forward finish, stop-------------------------------------

  //-----------------------------return to initial position-------------------------------------
  //	 if((count > 420) && (count < 440))  // return
  if ((not_walking) || (not_come_back))   // return

  {
    //std::cout<<"not walking"<<std::endl;
    if( step2 < STEP)    // walking forward
    {
      std::cout<<"not walking  "<<step2<<std::endl;
      step2++;
      legend_RF.Y = (1-(double)step2/STEP) * legend_pre_RF.Y;
      legend_RH.Y = (1-(double)step2/STEP) * legend_pre_RH.Y;
      legend_LM.Y = (1-(double)step2/STEP) * legend_pre_LM.Y;
      legend_RM.Y = (1-(double)step2/STEP) * legend_pre_RM.Y;
      legend_LF.Y = (1-(double)step2/STEP) * legend_pre_LF.Y;
      legend_LH.Y = (1-(double)step2/STEP) * legend_pre_LH.Y;

      legend_RF.Z = (1-(double)step2/STEP) * legend_pre_RF.Z;
      legend_RH.Z = (1-(double)step2/STEP) * legend_pre_RH.Z;
      legend_LM.Z = (1-(double)step2/STEP) * legend_pre_LM.Z;
      legend_RM.Z = (1-(double)step2/STEP) * legend_pre_RM.Z;
      legend_LF.Z = (1-(double)step2/STEP) * legend_pre_LF.Z;
      legend_LH.Z = (1-(double)step2/STEP) * legend_pre_LH.Z;
      step++;
    }

  }

  //---------------move the front leg to go ahead, the first time-------------------
  //	 if((count > 440) && (count<460))  //this means press go touching button
  if(lift_leg)  //this means press go touching button
  {
    std::cout<<"lifting"<<std::endl;
    if( step > 0)
    {
      step--;
      legend_RF.X = (1-(double)step/(STEP*50)) * desire.X;
      legend_RF.Y = (1-(double)step/(STEP*50)) * desire.Y;
      legend_RF.Z = (1-(double)step/(STEP*50)) * desire.Z;
    }
  }

  //	 //--------------front leg back to initial angle-------------------------------------
  ////	 if((count > 480) && (count<520))  //this means press return
  //	 if(come_back)  //this means press return
  //	 {
  //	 	 if( step < STEP)
  //	 	 {
  //	 	 	step++;
  //	 	 	legend_RF.X = (1-(double)step/STEP) * desire.X;
  //	 		legend_RF.Y = (1-(double)step/STEP) * desire.Y;
  //	 		legend_RF.Z = (1-(double)step/STEP) * desire.Z;
  //	 	 }
  //	 }
  //
  //	 //---------------move the front leg to go ahead, the second time-------------------
  //	 if((count > 520) && (count<540))  //this means press go touching button
  //	 {
  //	 	if( step > 0)
  //	 	{
  //	 		step--;
  //	 		legend_RF.X = (1-(double)step/STEP) * desire.X;
  //			legend_RF.Y = (1-(double)step/STEP) * desire.Y;
  //			legend_RF.Z = (1-(double)step/STEP) * desire.Z;
  //	 	}
  //	 }

  //--------------front leg back to initial angle again-------------------------------------
  //	 if((count > 540) && (count<560))  //this means press return
  if(lower_leg)
  {
    std::cout<<"lowering"<<std::endl;
    if( step < STEP)
    {
      step++;
      legend_RF.X = (1-(double)step/STEP) * desire.X;
      legend_RF.Y = (1-(double)step/STEP) * desire.Y;
      legend_RF.Z = (1-(double)step/STEP) * desire.Z;
    }
  }

  //--------------------------start to move backward------------------------------------------
  //	 if (count > 600)
  if (come_back)
  {

    //-----------------walking backwards------------------------------------------------------------//
    std::cout<<"coming back"<<std::endl;
    if(swingflag)
    {
      legend_RF.Y = legend_RH.Y = legend_LM.Y = -((double)n++/MAXNUM) * STRIDE;
      legend_RF.Z = legend_RH.Z = legend_LM.Z = - (double) abs(n) * 2 * LEGH / MAXNUM + LEGH;
      legend_LF.Y = legend_LH.Y = legend_RM.Y = ((double)n/MAXNUM) * STRIDE;
      legend_LF.Z = legend_LH.Z = legend_RM.Z = 0;
      if((MAXNUM/2) == n)	 swingflag = false;
    }
    else
    {
      legend_RF.Y = legend_RH.Y = legend_LM.Y = -((double)n--/MAXNUM) * STRIDE;
      legend_RF.Z = legend_RH.Z = legend_LM.Z = 0;
      legend_LF.Y = legend_LH.Y = legend_RM.Y = ((double)n/MAXNUM) * STRIDE;
      legend_LF.Z = legend_LH.Z = legend_RM.Z = - (double) abs(n) * 2 * LEGH / MAXNUM + LEGH;
      if((-MAXNUM/2) == n)	 swingflag = true;
    }
    step2 = 0;


    //-----------------walking backwards to left------------------------------------------------------------//
    //		 std::cout<<"coming back left"<<std::endl;
    //		 if(swingflag)
    //		 {
    //		 	 legend_RF.Y = legend_RH.Y = legend_LM.Y = -((double)n++/MAXNUM) * STRIDE;
    //		 	 legend_RF.Y = legend_RH.Y = -((double)n++/MAXNUM) * STRIDE*0.1;
    //		 	 legend_RF.Z = legend_RH.Z = legend_LM.Z = - (double) abs(n) * 2 * LEGH / MAXNUM + LEGH;
    //
    //		 	 legend_LF.Y = legend_LH.Y = legend_RM.Y = ((double)n/MAXNUM) * STRIDE;
    //			 legend_RM.Y = ((double)n/MAXNUM) * STRIDE*0.1;
    //			 legend_LF.Z = legend_LH.Z = legend_RM.Z = 0;
    //
    //			 if((MAXNUM/2) == n)	 swingflag = false;
    //		 }
    //		 else
    //		 {
    //			 legend_RF.Y = legend_RH.Y = legend_LM.Y = -((double)n--/MAXNUM) * STRIDE;
    //			 legend_RF.Y = legend_RH.Y = -((double)n--/MAXNUM) * STRIDE*0.1;
    //			 legend_RF.Z = legend_RH.Z = legend_LM.Z = 0;
    //
    //			 legend_LF.Y = legend_LH.Y = legend_RM.Y = ((double)n/MAXNUM) * STRIDE;
    //			 legend_RM.Y = ((double)n/MAXNUM) * STRIDE*0.1;
    //			 legend_LF.Z = legend_LH.Z = legend_RM.Z = - (double) abs(n) * 2 * LEGH / MAXNUM + LEGH;
    //
    //			 if((-MAXNUM/2) == n)	 swingflag = true;
    //		 }
    //		 step2 = 0;


    //-----------------walking backwards to right------------------------------------------------------------//
    //		 std::cout<<"coming back right"<<std::endl;
    //		 if(swingflag)
    //		 {
    //		 	 legend_LM.Y = -((double)n++/MAXNUM) * STRIDE*0.1;
    //		 	 legend_RF.Y = legend_RH.Y = -((double)n++/MAXNUM) * STRIDE;
    //		 	 legend_RF.Z = legend_RH.Z = legend_LM.Z = - (double) abs(n) * 2 * LEGH / MAXNUM + LEGH;
    //
    //		 	 legend_LF.Y = legend_LH.Y = ((double)n/MAXNUM) * STRIDE*0.1;
    //			 legend_RM.Y = ((double)n/MAXNUM) * STRIDE;
    //			 legend_LF.Z = legend_LH.Z = legend_RM.Z = 0;
    //
    //			 if((MAXNUM/2) == n)	 swingflag = false;
    //		 }
    //		 else
    //		 {
    //			 legend_LM.Y = -((double)n--/MAXNUM) * STRIDE*0.1;
    //			 legend_RF.Y = legend_RH.Y = -((double)n--/MAXNUM) * STRIDE;
    //			 legend_RF.Z = legend_RH.Z = legend_LM.Z = 0;
    //
    //			 legend_LF.Y = legend_LH.Y = ((double)n/MAXNUM) * STRIDE*0.1;
    //			 legend_RM.Y = ((double)n/MAXNUM) * STRIDE;
    //			 legend_LF.Z = legend_LH.Z = legend_RM.Z = - (double) abs(n) * 2 * LEGH / MAXNUM + LEGH;
    //
    //			 if((-MAXNUM/2) == n)	 swingflag = true;
    //		 }
    //		 step2 = 0;

  }



  if (turn_left)
  {


    //-----------------turn left------------------------------------------------------------//
    std::cout<<"turn left"<<std::endl;
    if(swingflag)
    {
      legend_RF.Y = legend_RH.Y = ((double)n++/MAXNUM) * STRIDE;
      legend_LM.Y = ((double)n/MAXNUM) * STRIDE*0.05;
      legend_RF.Z = legend_RH.Z = legend_LM.Z = - (double) abs(n) * 2 * LEGH / MAXNUM + LEGH;

      legend_LF.Y = legend_LH.Y = legend_RM.Y = -((double)n/MAXNUM) * STRIDE;
      legend_LF.Y = legend_LH.Y = -((double)n/MAXNUM) * STRIDE*0.05;
      legend_LF.Z = legend_LH.Z = legend_RM.Z = 0;
      if((MAXNUM/2) == n)	 swingflag = false;
    }
    else
    {
      legend_RF.Y = legend_RH.Y = ((double)n--/MAXNUM) * STRIDE;
      legend_LM.Y = ((double)n/MAXNUM) * STRIDE*0.05;
      legend_RF.Z = legend_RH.Z = legend_LM.Z = 0;

      legend_LF.Y = legend_LH.Y = legend_RM.Y = -((double)n/MAXNUM) * STRIDE;
      legend_LF.Y = legend_LH.Y = -((double)n/MAXNUM) * STRIDE*0.05;
      legend_LF.Z = legend_LH.Z = legend_RM.Z = - (double) abs(n) * 2 * LEGH / MAXNUM + LEGH;
      if((-MAXNUM/2) == n)	 swingflag = true;
    }
    legend_pre_RF = legend_RF;
    legend_pre_RM = legend_RM;
    legend_pre_RH = legend_RH;
    legend_pre_LF = legend_LF;
    legend_pre_LM = legend_LM;
    legend_pre_LH = legend_LH;

    step2 = 0;

  }


  if (turn_right)
  {
    //-----------------turn right------------------------------------------------------------//
    std::cout<<"Turn right"<<std::endl;
    if(swingflag)
    {
      legend_LM.Y = ((double)n++/MAXNUM) * STRIDE*1.0;
      legend_RF.Y = legend_RH.Y = ((double)n/MAXNUM) * STRIDE*0.05;
      legend_RF.Z = legend_RH.Z = legend_LM.Z = - (double) abs(n) * 2 * LEGH / MAXNUM + LEGH;

      legend_LF.Y = legend_LH.Y = -((double)n/MAXNUM) * STRIDE*1.0;
      legend_RM.Y = -((double)n/MAXNUM) * STRIDE*0.05;
      legend_LF.Z = legend_LH.Z = legend_RM.Z = 0;

      if((MAXNUM/2) == n)	 swingflag = false;
    }
    else
    {
      legend_LM.Y = ((double)n--/MAXNUM) * STRIDE*1.0;
      legend_RF.Y = legend_RH.Y = ((double)n/MAXNUM) * STRIDE*0.05;
      legend_RF.Z = legend_RH.Z = legend_LM.Z = 0;

      legend_LF.Y = legend_LH.Y = legend_RM.Y = -((double)n/MAXNUM) * STRIDE*1.0;
      legend_RM.Y = ((double)n/MAXNUM) * STRIDE*0.05;
      legend_LF.Z = legend_LH.Z = legend_RM.Z = -(double) abs(n) * 2 * LEGH / MAXNUM + LEGH;

      if((-MAXNUM/2) == n)	 swingflag = true;
    }
    legend_pre_RF = legend_RF;
    legend_pre_RM = legend_RM;
    legend_pre_RH = legend_RH;
    legend_pre_LF = legend_LF;
    legend_pre_LM = legend_LM;
    legend_pre_LH = legend_LH;

    step2 = 0;

  }

  if (sideward_right)
  {
    //-----------------sideward right------------------------------------------------------------//
    std::cout<<"sideward right"<<std::endl;
    if(swingflag)
    {
      legend_RF.X = legend_RH.X = legend_LM.X = ((double)n++/MAXNUM) * STRIDE*0.6;
      legend_RF.Z = legend_RH.Z = legend_LM.Z = - (double) abs(n) * 2 * LEGH / MAXNUM + LEGH;
      legend_LF.X = legend_LH.X = legend_RM.X = -((double)n/MAXNUM) * STRIDE*0.6;
      legend_LF.Y = legend_LH.Y = legend_RM.Y = 0;
      if((MAXNUM/2) == n)	 swingflag = false;
    }
    else
    {
      legend_RF.X = legend_RH.X = legend_LM.X = ((double)n--/MAXNUM) * STRIDE*0.6;
      legend_RF.Y = legend_RH.Y = legend_LM.Y = 0;
      legend_LF.X = legend_LH.X = legend_RM.X = -((double)n/MAXNUM) * STRIDE*0.6;
      legend_LF.Z = legend_LH.Z = legend_RM.Z = - (double) abs(n) * 2 * LEGH / MAXNUM + LEGH;
      if((-MAXNUM/2) == n)	 swingflag = true;
    }
    legend_pre_RF = legend_RF;
    legend_pre_RM = legend_RM;
    legend_pre_RH = legend_RH;
    legend_pre_LF = legend_LF;
    legend_pre_LM = legend_LM;
    legend_pre_LH = legend_LH;

    step2 = 0;

  }

  if (sideward_left)
  {
    //-----------------sideward left------------------------------------------------------------//
    std::cout<<"sideward left"<<std::endl;
    if(swingflag)
    {
      legend_RF.X = legend_RH.X = legend_LM.X = -((double)n++/MAXNUM) * STRIDE*0.6;
      legend_RF.Z = legend_RH.Z = legend_LM.Z = - (double) abs(n) * 2 * LEGH / MAXNUM + LEGH;
      legend_LF.X = legend_LH.X = legend_RM.X = ((double)n/MAXNUM) * STRIDE*0.6;
      legend_LF.Y = legend_LH.Y = legend_RM.Y = 0;
      if((MAXNUM/2) == n)	 swingflag = false;
    }
    else
    {
      legend_RF.X = legend_RH.X = legend_LM.X = -((double)n--/MAXNUM) * STRIDE*0.6;
      legend_RF.Y = legend_RH.Y = legend_LM.Y = 0;
      legend_LF.X = legend_LH.X = legend_RM.X = ((double)n/MAXNUM) * STRIDE*0.6;
      legend_LF.Z = legend_LH.Z = legend_RM.Z = - (double) abs(n) * 2 * LEGH / MAXNUM + LEGH;
      if((-MAXNUM/2) == n)	 swingflag = true;
    }
    legend_pre_RF = legend_RF;
    legend_pre_RM = legend_RM;
    legend_pre_RH = legend_RH;
    legend_pre_LF = legend_LF;
    legend_pre_LM = legend_LM;
    legend_pre_LH = legend_LH;

    step2 = 0;


  }

  if (turnright_sideward)
  {
    //		 //-----------------turn right with sideward------------------------------------------------------------//
    std::cout<<"Turn right with sideward"<<std::endl;
    if(swingflag)
    {
      legend_LM.Y = ((double)n++/MAXNUM) * STRIDE*1.0;
      legend_RF.Y = legend_RH.Y = ((double)n/MAXNUM) * STRIDE*0.05;
      legend_RF.X = legend_RH.X = legend_LM.X = ((double)n/MAXNUM) * STRIDE*0.6;
      legend_RF.Z = legend_RH.Z = legend_LM.Z = - (double) abs(n) * 2 * LEGH / MAXNUM + LEGH;
      legend_LF.X = legend_LH.X = legend_RM.X = -((double)n/MAXNUM) * STRIDE*0.6;

      legend_LF.Y = legend_LH.Y = -((double)n/MAXNUM) * STRIDE*1.0;
      legend_RM.Y = -((double)n/MAXNUM) * STRIDE*0.05;
      legend_LF.Z = legend_LH.Z = legend_RM.Z = 0;

      if((MAXNUM/2) == n)	 swingflag = false;
    }
    else
    {
      legend_LM.Y = ((double)n--/MAXNUM) * STRIDE*1.0;
      legend_RF.Y = legend_RH.Y = ((double)n/MAXNUM) * STRIDE*0.05;
      legend_RF.Z = legend_RH.Z = legend_LM.Z = 0;
      legend_RF.X = legend_RH.X = legend_LM.X = ((double)n/MAXNUM) * STRIDE*0.6;

      legend_LF.Y = legend_LH.Y = legend_RM.Y = -((double)n/MAXNUM) * STRIDE*1.0;
      legend_RM.Y = ((double)n/MAXNUM) * STRIDE*0.05;
      legend_LF.X = legend_LH.X = legend_RM.X = -((double)n/MAXNUM) * STRIDE*0.6;
      legend_LF.Z = legend_LH.Z = legend_RM.Z = -(double) abs(n) * 2 * LEGH / MAXNUM + LEGH;

      if((-MAXNUM/2) == n)	 swingflag = true;
    }
    legend_pre_RF = legend_RF;
    legend_pre_RM = legend_RM;
    legend_pre_RH = legend_RH;
    legend_pre_LF = legend_LF;
    legend_pre_LM = legend_LM;
    legend_pre_LH = legend_LH;

    step2 = 0;

  }

  if (turnleft_sideward)
  {

    //-----------------turn left with sideward------------------------------------------------------------//
    std::cout<<"turn left with sideward "<<std::endl;
    if(swingflag)
    {
      legend_RF.Y = legend_RH.Y = legend_LM.Y = ((double)n++/MAXNUM) * STRIDE;
      legend_LM.Y = ((double)n/MAXNUM) * STRIDE*0.05;
      legend_RF.X = legend_RH.X = legend_LM.X = -((double)n/MAXNUM) * STRIDE*0.6;
      legend_RF.Z = legend_RH.Z = legend_LM.Z = - (double) abs(n) * 2 * LEGH / MAXNUM + LEGH;

      legend_LF.Y = legend_LH.Y = legend_RM.Y = -((double)n/MAXNUM) * STRIDE;
      legend_LF.X = legend_LH.X = legend_RM.X = ((double)n/MAXNUM) * STRIDE*0.6;
      legend_LF.Y = legend_LH.Y = -((double)n/MAXNUM) * STRIDE*0.05;
      legend_LF.Z = legend_LH.Z = legend_RM.Z = 0;
      if((MAXNUM/2) == n)	 swingflag = false;


    }
    else
    {
      legend_RF.Y = legend_RH.Y = legend_LM.Y = ((double)n--/MAXNUM) * STRIDE;
      legend_RF.X = legend_RH.X = legend_LM.X = -((double)n/MAXNUM) * STRIDE*0.6;
      legend_LM.Y = ((double)n/MAXNUM) * STRIDE*0.05;
      legend_RF.Z = legend_RH.Z = legend_LM.Z = 0;

      legend_LF.Y = legend_LH.Y = legend_RM.Y = -((double)n/MAXNUM) * STRIDE;
      legend_LF.Y = legend_LH.Y = -((double)n/MAXNUM) * STRIDE*0.05;
      legend_LF.X = legend_LH.X = legend_RM.X = ((double)n/MAXNUM) * STRIDE*0.6;
      legend_LF.Z = legend_LH.Z = legend_RM.Z = - (double) abs(n) * 2 * LEGH / MAXNUM + LEGH;
      if((-MAXNUM/2) == n)	 swingflag = true;



    }
    legend_pre_RF = legend_RF;
    legend_pre_RM = legend_RM;
    legend_pre_RH = legend_RH;
    legend_pre_LF = legend_LF;
    legend_pre_LM = legend_LM;
    legend_pre_LH = legend_LH;

    step2 = 0;


  }

  assert(legend_RF.X==legend_RF.X);
  assert(legend_RF.Y==legend_RF.Y);
  assert(legend_RF.Z==legend_RF.Z);
  assert(legend_RM.X==legend_RM.X);
  assert(legend_RM.Y==legend_RM.Y);
  assert(legend_RM.Z==legend_RM.Z);
  assert(legend_RH.X==legend_RH.X);
  assert(legend_RH.Y==legend_RH.Y);
  assert(legend_RH.Z==legend_RH.Z);
  assert(legend_LF.X==legend_LF.X);
  assert(legend_LF.Y==legend_LF.Y);
  assert(legend_LF.Z==legend_LF.Z);
  assert(legend_LM.X==legend_LM.X);
  assert(legend_LM.Y==legend_LM.Y);
  assert(legend_LM.Z==legend_LM.Z);
  assert(legend_LH.X==legend_LH.X);
  assert(legend_LH.Y==legend_LH.Y);
  assert(legend_LH.Z==legend_LH.Z);

  //--------------control the six legs-------------------------
  SingleLegInverKine_RF(legend_RF.X,legend_RF.Y,legend_RF.Z);
  SingleLegInverKine_RM(legend_RM.X,legend_RM.Y,legend_RM.Z);
  SingleLegInverKine_RH(legend_RH.X,legend_RH.Y,legend_RH.Z);
  SingleLegInverKine_LF(legend_LF.X,legend_LF.Y,legend_LF.Z);
  SingleLegInverKine_LM(legend_LM.X,legend_LM.Y,legend_LM.Z);
  SingleLegInverKine_LH(legend_LH.X,legend_LH.Y,legend_LH.Z);

  //--------------------mapping to -1 ~ 1 -----------------------

  assert(RF.TC==RF.TC);
  assert(RF.CTr==RF.CTr);
  assert(RF.FTi==RF.FTi);
  assert(RM.TC==RM.TC);
  assert(RM.CTr==RM.CTr);
  assert(RM.FTi==RM.FTi);
  assert(RH.TC==RH.TC);
  assert(RH.CTr==RH.CTr);
  assert(RH.FTi==RH.FTi);
  assert(LF.TC==LF.TC);
  assert(LF.CTr==LF.CTr);
  assert(LF.FTi==LF.FTi);
  assert(LM.TC==LM.TC);
  assert(LM.CTr==LM.CTr);
  assert(LM.FTi==LM.FTi);
  assert(LH.TC==LH.TC);
  assert(LH.CTr==LH.CTr);
  assert(LH.FTi==LH.FTi);

  out.at(TR0_m) = RF.TC * 0.0143 * 180 / PI;
  out.at(TR1_m) = RM.TC * 0.0167 * 180 / PI;
  out.at(TR2_m) = RH.TC * 0.0143 * 180 / PI;

  out.at(TL0_m) = -LF.TC * 0.0143 * 180 / PI;
  out.at(TL1_m) = -LM.TC * 0.0167 * 180 / PI;
  out.at(TL2_m) = -LH.TC * 0.0143 * 180 / PI;

  out.at(CL0_m) = -LF.CTr * 0.0143 * 180 / PI;
  out.at(CL1_m) = -LM.CTr * 0.0143 * 180 / PI;
  out.at(CL2_m) = -LH.CTr * 0.0143 * 180 / PI;

  out.at(CR0_m) = RF.CTr * 0.0143 * 180 / PI;
  out.at(CR1_m) = RM.CTr * 0.0143 * 180 / PI;
  out.at(CR2_m) = RH.CTr * 0.0143 * 180 / PI;

  out.at(FR0_m) = RF.FTi * 0.0182 * 180 / PI + 1.3636;
  out.at(FR1_m) = RM.FTi * 0.0182 * 180 / PI + 1.3636;
  out.at(FR2_m) = RH.FTi * 0.0182 * 180 / PI + 1.3636;

  out.at(FL0_m) = -LF.FTi * 0.0182 * 180 / PI + 1.3636;
  out.at(FL1_m) = -LM.FTi * 0.0182 * 180 / PI + 1.3636;
  out.at(FL2_m) = -LH.FTi * 0.0182 * 180 / PI + 1.3636;

  count++;

}

void KineController::SingleLegInverKine_RF(double x,double y,double z)
{
  double L;
  Point pA_b;
  Point pD_b;

  pA_b.X = ( l1 + l2 * cos( RF_ini.CTr ) + l3 * cos( - RF_ini.FTi - RF_ini.CTr ) ) * cos(RF_ini.TC) + x;  //X coordinate
  pA_b.Y = ( l1 + l2 * cos( RF_ini.CTr ) + l3 * cos( - RF_ini.FTi - RF_ini.CTr ) ) * sin(RF_ini.TC) + y; //Y coordinate
  pA_b.Z = l2 * sin( RF_ini.CTr ) - l3 * sin( - RF_ini.FTi - RF_ini.CTr ) + z; //Z coordinate "causes nan"


  RF.TC = atan( pA_b.Y / pA_b.X );  //angle of TC- joint

  // TODO: this is a temporary solution for filtering out nan's, do a better calculation/additional transition when changing gaits
  if (RF.TC!=RF.TC){
    RF.TC = RF_old.TC;
    std::cout<<"nan occured at joint RF.TC"<<std::endl;
  }

  pD_b.X = l1 * cos( RF.TC );
  pD_b.Y = l1 * sin( RF.TC );
  pD_b.Z = 0;

  //H = l3 * sin( -RF_ini.FTi - RF_ini.CTr ) - l2 * sin( RF_ini.CTr ); //height of body

  L = sqrt( (pA_b.X-pD_b.X)*(pA_b.X-pD_b.X) + (pA_b.Y-pD_b.Y)*(pA_b.Y-pD_b.Y) + (pA_b.Z-pD_b.Z)*(pA_b.Z-pD_b.Z) );

  RF.FTi = acos( ( l2*l2 + l3*l3 - L*L ) / ( 2 * l2 * l3 ) ) - PI; //angle of FTi- joint

  // TODO: this is a temporary solution for filtering out nan's, do a better calculation/additional transition when changing gaits
  if (RF.FTi!=RF.FTi){
    RF.FTi = RF_old.FTi;
    std::cout<<"nan occured at joint RF.FTi"<<std::endl;
  }

  pA_pD_Z_RF = (pA_b.Z - pD_b.Z) / L;


  RF.CTr = acos( ( L*L + l2*l2 - l3*l3 ) / ( 2 * L * l2 ) ) + asin(pA_pD_Z_RF); //angle of CTr- joint

  // TODO: this is a temporary solution for filtering out nan's, do a better calculation/additional transition when changing gaits
  if (RF.CTr!=RF.CTr){
    RF.CTr = RF_old.CTr;
    std::cout<<"nan occured at joint RF.CTr"<<std::endl;
  }

  RF_old = RF;
}

void KineController::SingleLegInverKine_RM(double x,double y,double z)
{
  double L;
  Point pA_b;
  Point pD_b;

  pA_b.X = ( l1 + l2 * cos( RM_ini.CTr ) + l3 * cos( - RM_ini.FTi - RM_ini.CTr ) ) * cos(RM_ini.TC) + x;  //X coordinate
  pA_b.Y = ( l1 + l2 * cos( RM_ini.CTr ) + l3 * cos( - RM_ini.FTi - RM_ini.CTr ) ) * sin(RM_ini.TC) + y; //Y coordinate
  pA_b.Z = l2 * sin( RM_ini.CTr ) - l3 * sin( - RM_ini.FTi - RM_ini.CTr ) + z; //Z coordinate  ***Problem Still

  RM.TC = atan( pA_b.Y / pA_b.X );  //angle of TC- joint

  // TODO: this is a temporary solution for filtering out nan's, do a better calculation/additional transition when changing gaits
  if (RM.TC!=RM.TC){
    RM.TC = RM_old.TC;
    std::cout<<"nan occured at joint RM.TC"<<std::endl;
  }


  pD_b.X = l1 * cos( RM.TC );
  pD_b.Y = l1 * sin( RM.TC );
  pD_b.Z = 0;

  //H = l3 * sin( -RF_ini.FTi - RF_ini.CTr ) - l2 * sin( RF_ini.CTr ); //height of body

  L = sqrt( (pA_b.X-pD_b.X)*(pA_b.X-pD_b.X) + (pA_b.Y-pD_b.Y)*(pA_b.Y-pD_b.Y) + (pA_b.Z-pD_b.Z)*(pA_b.Z-pD_b.Z) );

  RM.FTi = acos( ( l2*l2 + l3*l3 - L*L ) / ( 2 * l2 * l3 ) ) - PI; //angle of FTi- joint

  // TODO: this is a temporary solution for filtering out nan's, do a better calculation/additional transition when changing gaits
  if (RM.FTi!=RM.FTi){
    RM.FTi = RM_old.FTi;
    std::cout<<"nan occured at joint RM.FTi"<<std::endl;
  }

  pA_pD_Z_RM = (pA_b.Z - pD_b.Z) / L;


  RM.CTr = acos( ( L*L + l2*l2 - l3*l3 ) / ( 2 * L * l2 ) ) + asin(pA_pD_Z_RM); //angle of CTr- joint


  // TODO: this is a temporary solution for filtering out nan's, do a better calculation/additional transition when changing gaits
  if (RM.CTr!=RM.CTr){
    RM.CTr = RM_old.CTr;
    std::cout<<"nan occured at joint RM.CTr"<<std::endl;
  }

  RM_old = RM;
}

void KineController::SingleLegInverKine_RH(double x,double y,double z)
{
  double L;
  Point pA_b;
  Point pD_b;

  pA_b.X = ( l1 + l2 * cos( RH_ini.CTr ) + l3 * cos( - RH_ini.FTi - RH_ini.CTr ) ) * cos(RH_ini.TC) + x;  //X coordinate
  pA_b.Y = ( l1 + l2 * cos( RH_ini.CTr ) + l3 * cos( - RH_ini.FTi - RH_ini.CTr ) ) * sin(RH_ini.TC) + y; //Y coordinate
  pA_b.Z = l2 * sin( RH_ini.CTr ) - l3 * sin( - RH_ini.FTi - RH_ini.CTr ) + z; //Z coordinate  ***Problem Still

  RH.TC = atan( pA_b.Y / pA_b.X );  //angle of TC- joint

  // TODO: this is a temporary solution for filtering out nan's, do a better calculation/additional transition when changing gaits
  if (RH.TC!=RH.TC){
    RH.TC = RH_old.TC;
    std::cout<<"nan occured at joint RH.TC"<<std::endl;
  }


  pD_b.X = l1 * cos( RH.TC );
  pD_b.Y = l1 * sin( RH.TC );
  pD_b.Z = 0;

  //H = l3 * sin( -RF_ini.FTi - RF_ini.CTr ) - l2 * sin( RF_ini.CTr ); //height of body

  L = sqrt( (pA_b.X-pD_b.X)*(pA_b.X-pD_b.X) + (pA_b.Y-pD_b.Y)*(pA_b.Y-pD_b.Y) + (pA_b.Z-pD_b.Z)*(pA_b.Z-pD_b.Z) );

  RH.FTi = acos( ( l2*l2 + l3*l3 - L*L ) / ( 2 * l2 * l3 ) ) - PI; //angle of FTi- joint

  // TODO: this is a temporary solution for filtering out nan's, do a better calculation/additional transition when changing gaits
  if (RH.FTi!=RH.FTi){
    RH.FTi = RH_old.FTi;
    std::cout<<"nan occured at joint RH.FTi"<<std::endl;
  }

  pA_pD_Z_RH = (pA_b.Z - pD_b.Z) / L;

  RH.CTr = acos( ( L*L + l2*l2 - l3*l3 ) / ( 2 * L * l2 ) ) + asin(pA_pD_Z_RH); //angle of CTr- joint

  // TODO: this is a temporary solution for filtering out nan's, do a better calculation/additional transition when changing gaits
  if (RH.CTr!=RH.CTr){
    RH.CTr = RH_old.CTr;
    std::cout<<"nan occured at joint RH.CTr"<<std::endl;
  }

  RH_old = RH;
}

void KineController::SingleLegInverKine_LF(double x,double y,double z)
{
  double L;
  Point pA_b;
  Point pD_b;

  pA_b.X = ( -l1 - l2 * cos( LF_ini.CTr ) - l3 * cos( LF_ini.FTi + LF_ini.CTr ) ) * cos(LF_ini.TC) + x;  //X coordinate
  pA_b.Y = ( -l1 - l2 * cos( LF_ini.CTr ) - l3 * cos( LF_ini.FTi + LF_ini.CTr ) ) * sin(LF_ini.TC) + y; //Y coordinate
  pA_b.Z = - l2 * sin(LF_ini.CTr ) - l3 * sin( LF_ini.FTi + LF_ini.CTr ) + z; //Z coordinate  ***Problem Still

  LF.TC = atan( pA_b.Y / pA_b.X );  //angle of TC- joint

  // TODO: this is a temporary solution for filtering out nan's, do a better calculation/additional transition when changing gaits
  if (LF.TC!=LF.TC){
    std::cout<<"nan occured at joint LF.TC"<<std::endl;
    std::cout<<"LF.TC="<<LF.TC<<"  sign="<<signbit(LF.TC)<<std::endl;
    LF.TC = LF_old.TC;
  }

  pD_b.X = -l1 * cos( LF.TC );
  pD_b.Y = -l1 * sin( LF.TC );
  pD_b.Z = 0;

  //H = l3 * sin( -RF_ini.FTi - RF_ini.CTr ) - l2 * sin( RF_ini.CTr ); //height of body

  L = sqrt( (pA_b.X-pD_b.X)*(pA_b.X-pD_b.X) + (pA_b.Y-pD_b.Y)*(pA_b.Y-pD_b.Y) + (pA_b.Z-pD_b.Z)*(pA_b.Z-pD_b.Z) );

  LF.FTi = PI - acos( ( l2*l2 + l3*l3 - L*L ) / ( 2 * l2 * l3 ) ); //angle of FTi- joint

  // TODO: this is a temporary solution for filtering out nan's, do a better calculation/additional transition when changing gaits
  if (LF.FTi!=LF.FTi){
    std::cout<<"nan occured at joint LF.FTi"<<std::endl;
    std::cout<<"LF.FTi="<<LF.FTi<<"  sign="<<signbit(LF.FTi)<<std::endl;
    LF.FTi = LF_old.FTi;
  }

  pA_pD_Z_LF = (pA_b.Z - pD_b.Z) / L;

  LF.CTr =  - acos( ( L*L + l2*l2 - l3*l3 ) / ( 2 * L * l2 ) ) - asin(pA_pD_Z_LF); //angle of CTr- joint

  // TODO: this is a temporary solution for filtering out nan's, do a better calculation/additional transition when changing gaits
  if (LF.CTr!=LF.CTr){
    std::cout<<"nan occured at joint LF.CTr"<<std::endl;
    std::cout<<"LF.CTr="<<LF.CTr<<"  sign="<<signbit(LF.CTr)<<std::endl;
    LF.CTr = LF_old.CTr;
  }

  LF_old = LF;
}

void KineController::SingleLegInverKine_LM(double x,double y,double z)
{
  double L;
  Point pA_b;
  Point pD_b;

  pA_b.X = ( -l1 - l2 * cos( LM_ini.CTr ) - l3 * cos( LM_ini.FTi + LM_ini.CTr ) ) * cos(LM_ini.TC) + x;  //X coordinate
  pA_b.Y = ( -l1 - l2 * cos( LM_ini.CTr ) - l3 * cos( LM_ini.FTi + LM_ini.CTr ) ) * sin(LM_ini.TC) + y; //Y coordinate
  pA_b.Z = - l2 * sin( LM_ini.CTr ) - l3 * sin( LM_ini.FTi + LM_ini.CTr ) + z; //Z coordinate  ***Problem Still

  LM.TC = atan( pA_b.Y / pA_b.X );  //angle of TC- joint

  // TODO: this is a temporary solution for filtering out nan's, do a better calculation/additional transition when changing gaits
  if (LM.TC!=LM.TC){
    LM.TC = LM_old.TC;
    std::cout<<"nan occured at joint LM.TC"<<std::endl;
  }


  pD_b.X = -l1 * cos( LM.TC );
  pD_b.Y = -l1 * sin( LM.TC );
  pD_b.Z = 0;

  //H = l3 * sin( -RF_ini.FTi - RF_ini.CTr ) - l2 * sin( RF_ini.CTr ); //height of body

  L = sqrt( (pA_b.X-pD_b.X)*(pA_b.X-pD_b.X) + (pA_b.Y-pD_b.Y)*(pA_b.Y-pD_b.Y) + (pA_b.Z-pD_b.Z)*(pA_b.Z-pD_b.Z) );

  LM.FTi = PI - acos( ( l2*l2 + l3*l3 - L*L ) / ( 2 * l2 * l3 ) ); //angle of FTi- joint

  // TODO: this is a temporary solution for filtering out nan's, do a better calculation/additional transition when changing gaits
  if (LM.FTi!=LM.FTi){
    LM.FTi = LM_old.FTi;
    std::cout<<"nan occured at joint LM.FTi"<<std::endl;
  }

  pA_pD_Z_LM = (pA_b.Z - pD_b.Z) / L;

  LM.CTr =  - acos( ( L*L + l2*l2 - l3*l3 ) / ( 2 * L * l2 ) ) - asin(pA_pD_Z_LM); //angle of CTr- joint

  // TODO: this is a temporary solution for filtering out nan's, do a better calculation/additional transition when changing gaits
  if (LM.CTr!=LM.CTr){
    LM.CTr = LM_old.CTr;
    std::cout<<"nan occured at joint LM.CTr"<<std::endl;
  }

  LM_old = LM;
}

void KineController::SingleLegInverKine_LH(double x,double y,double z)
{
  double L;
  Point pA_b;
  Point pD_b;

  pA_b.X = ( -l1 - l2 * cos( LH_ini.CTr ) - l3 * cos( LH_ini.FTi + LH_ini.CTr ) ) * cos(LH_ini.TC) + x;  //X coordinate
  pA_b.Y = ( -l1 - l2 * cos( LH_ini.CTr ) - l3 * cos( LH_ini.FTi + LH_ini.CTr ) ) * sin(LH_ini.TC) + y; //Y coordinate
  pA_b.Z = - l2 * sin( LH_ini.CTr ) - l3 * sin( LH_ini.FTi + LH_ini.CTr ) + z; //Z coordinate  ***Problem Still

  LH.TC = atan( pA_b.Y / pA_b.X );  //angle of TC- joint

  // TODO: this is a temporary solution for filtering out nan's, do a better calculation/additional transition when changing gaits
  if (LH.TC!=LH.TC){
    LH.TC = LH_old.TC;
    std::cout<<"nan occured at joint LH.TC"<<std::endl;
  }

  pD_b.X = -l1 * cos( LH.TC );
  pD_b.Y = -l1 * sin( LH.TC );
  pD_b.Z = 0;

  //H = l3 * sin( -RF_ini.FTi - RF_ini.CTr ) - l2 * sin( RF_ini.CTr ); //height of body

  L = sqrt( (pA_b.X-pD_b.X)*(pA_b.X-pD_b.X) + (pA_b.Y-pD_b.Y)*(pA_b.Y-pD_b.Y) + (pA_b.Z-pD_b.Z)*(pA_b.Z-pD_b.Z) );

  LH.FTi = PI - acos( ( l2*l2 + l3*l3 - L*L ) / ( 2 * l2 * l3 ) ); //angle of FTi- joint

  // TODO: this is a temporary solution for filtering out nan's, do a better calculation/additional transition when changing gaits
  if (LH.FTi!=LH.FTi){
    LH.FTi = LH_old.FTi;
    std::cout<<"nan occured at joint LH.FTi"<<std::endl;
  }

  pA_pD_Z_LH = (pA_b.Z - pD_b.Z) / L;

  LH.CTr =  - acos( ( L*L + l2*l2 - l3*l3 ) / ( 2 * L * l2 ) ) - asin(pA_pD_Z_LH); //angle of CTr- joint

  // TODO: this is a temporary solution for filtering out nan's, do a better calculation/additional transition when changing gaits
  if (LH.CTr!=LH.CTr){
    LH.CTr = LH_old.CTr;
    std::cout<<"nan occured at joint LH.CTr"<<std::endl;
  }

  LH_old = LH;
}

