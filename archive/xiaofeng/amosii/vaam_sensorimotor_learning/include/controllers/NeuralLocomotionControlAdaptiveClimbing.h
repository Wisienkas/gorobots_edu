/*
 * NeuralLocomotionControlAdaptiveClimbing.h
 *
 *  Created on: May 2, 2011
 *      Author: poramate and xiong
 */

#ifndef NEURALLOCOMOTIONCONTROLADAPTIVECLIMBING_H_
#define NEURALLOCOMOTIONCONTROLADAPTIVECLIMBING_H_

#include <vector>
#include <cmath>
//#include <selforg/amosiisensormotordefinition.h>
#include <ode_robots/amosiisensormotordefinition.h>
//#include "sensor_motor_definition.h"

//Save files /read file
#include <iostream>
#include <fstream>
#include <string.h>

//atof function
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
//#include <selforg/controller_misc.h>
#include <time.h>
#include <sys/time.h>

#define PYTHONUSED 1 // 1 is preferred for the muscle model
#define WHMUSCLEALLPHA 0 // 0 is preferred for the muscle model with raw foot contact force,1 is the muscle model with expected foot contact force.
#define PURENEURON 0 // 0 is preferred for the muscle model

#define VELTIME 10000//20
#define STNEUACT -0.6
#define NUMFOOT 6
#define NUMMOTOR 19
#define PI 3.14159265358979
#define PI_DEG 180.0
#define TIMECOUNT 5000
#define CRMAXOUT 1
#define CRMINOUT -1
#define CRMAXANG ((-100.0/180.0)*3.14159265358979)//((100.0/180.0)*3.14159265358979)
#define CRMINANG ((45.0/180.0)*3.14159265358979)//((-30.0/180.0)*3.14159265358979)
#define FRMAXOUT 1
#define FRMINOUT -1
#define FRMAXANG ((55.0/180.0)*3.14159265358979)//((75.0/180.0)*3.14159265358979)
#define FRMINANG ((-70.0/180.0)*3.14159265358979)//((-50.0/180.0)*3.14159265358979)
#define FSERR 0.15
#define TESTPERIOD 6//6//3
#define KMAXRANG 2201//100//30
#define KSTEP 20//100//30
#define INIK 1
#define K2MAXRANG 50//4000//100//30
#define K2STEP 1//4000//100//30
#define INIK2 1

#define MINCTROUT 0.71
#define MINFTIOUT -1.0

#define CTRSTIFFSETP 0.5//1//1 for s = 0.04
#define TESTSTIFF 0//0

#define TESTSPEED 1

#define FCERROR 0.2//0.26//0.26 for s = 0.04

#define ALPHA 0.2//0.39
#define MAXCTRSTIFF 20

#define INICTRSTIFF 20.0
#define INIFTISTIFF 20.0
#define RLSLAMDA 0.99998//0.9998
#define DRLERROR 0.01//0.05//0.1
#define DRLFRATE 0.1
#define DRLRATE 0.2//9.99//0.46//0.2//0.46
#define LTDRLFCERROR 0.18
//0.2 is NOT a proper value for DRL (i.e.,  LEARNINGTYPE = 2), but 0.46 is ok for DRL.

#define LEARNINGTYPE 9
/*
 * 0 is a LMS learning over every stance phase.
 * 1 is a RLS learning over every time step.
 * 2 is a DRL over every time step.
 * 3 is an extended DRL over time step.
 * 4 is a fast learner same as that in the DRL.
 * 5 is a slow learner same as that in the DRL.
 * 6 is the extended DRL with parameter learning over time step.
 *
 * 7 is a pure dual rate learning over every time step.
 * 8 is a extented dual rate learning with parameter learning over every phase.
 */



#define TDRLEQU 5
/*
 * 0 is an old equation used in the extended DRL, performance is best.
 * 1 is derived by 0.
 * 2 is the equation from reza`s paper.
 * 3 is the equation from wolpert`s paer.
 * 4 is a new equation derived by 0.
 *
 * 5 is a new equation where the paramaters were taken from reza`s paper.
 */

#define CTRBASELINE 3.5
#define FTIBASELINE 1.5



//#define XDISFILE "XDis014.txt"
//#define YDISFILE "YDis014.txt"
//#define MAXZFILE "MaxZ014.txt"
//#define MINZFILE "MinZ014.txt"
//#define DIFFZFILE "DiffZ014.txt"
//#define K1FILE "K1.txt"
//#define K2FILE "K2.txt"
//#define CRMinFILE "CRMin.txt"
//#define CRMaxFILE "CRMax.txt"
//#define FRMinFILE "FRMin.txt"
//#define FRMaxFILE "FRMax.txt"
//#define DISFILE "Dis.txt"
#define ISSAVINGTEST 0
#define SPONRECOVE 0
#define SAVINGTESTTIME 800
#define ISRECORD 0
#define RTIMEFILE "RealTime.txt"
#define R0CTRSTIFFFILE "R0CtrStiffness.txt"
#define R1CTRSTIFFFILE "R1CtrStiffness.txt"
#define R2CTRSTIFFFILE "R2CtrStiffness.txt"
#define L0CTRSTIFFFILE "L0CtrStiffness.txt"
#define L1CTRSTIFFFILE "L1CtrStiffness.txt"
#define L2CTRSTIFFFILE "L2CtrStiffness.txt"


using namespace std;
//Save files

typedef struct{
    double Pos;
    double Vel;
}MusclePara;

typedef struct{
    double CJPos;
    double CJVel;
    double CJNeu;
    double FJPos;
    double FJVel;
    double FJNeu;
}MuscleOut;

typedef struct{
    double Neu;
    double PreNeu;
    double SampleTime;
    double Fc;
    double PreFc;
    double Radius;
}MuscleIn;

typedef struct{
    double PrePos;
    double PreVel;
    double PreNeu;
    double Neu;
    double K;
    double D;
    double MusFac;
    double MusLeng;
    double MaxNeu;
    double MinNeu;
    double MaxAng;
    double MinAng;
}JPara;

//3) Class for Neural locomotion control------

class NeuralLocomotionControlAdaptiveClimbing{

public:
    //VirtualMuscles(edited by lenonxiong)

      float Mass;
      float KneeLength;
      float HipLength;
      float Radius;
      
      unsigned int iTimeCount;
      
      ofstream oCTrStiff[6];
      ofstream oRealTime;

      float K1[NUMFOOT];
      float K2[NUMFOOT];
      float K11;
      float K22;
      float KPlusHind;
      float K3;

      float D1[NUMFOOT];
      float D2[NUMFOOT];
      float D11;
      float D22;
      float D3;
      
      float CJMaxOut;
      float CJMinOut;
      float CJMaxAng;
      float CJMinAng;
      
      float FJMaxOut;
      float FJMinOut;
      float FJMaxAng;
      float FJMinAng;
      
      float FMusclActFac;
      float CMusclActFac;
      
      float MaxMuscleAct;
      float MinMuscleAct;
      

      
      float MaxCJointAct;
      float MinCJointAct;
      float MaxFJointAct;
      float MinFJointAct;
      
      
      float MaxScaFoot;
      float MinScafoot;
      float MaxFootSim;
      float MinFootSim;
      
      float FMaxNeuOut;
      float FMinNeuOut;
      
      float CMaxNeuOut;
      float CMinNeuOut;
      
      
      float MaxJointAng_Deg[NUMMOTOR];
      float MinJointAng_Deg[NUMMOTOR];
      float MaxJointAng[NUMMOTOR];
      float MinJointAng[NUMMOTOR];
      
      std::vector<double> DegToMotor;
      
      
      float SampleTime;// = 0.1;
      
      float ScaExCjFoot;
      float ScaExFjFoot;
      double dHeight;
      
      double BodyX;
      double BodyY;
      double BodyZ;
      double PreBodyX;
      double PreBodyY;
      double XSpeed;
      double YSpeed;
      double SpeedTime;
      bool bReset;

      bool check_bReset(){
        return bReset;
      }



      double AveFC;
      double SumFC;

      int nSpeeCount;


      
      
      float MaxFoot[NUMFOOT];
      float MinFoot[NUMFOOT];
      
      /*
      float MaxJointAng[NUMMOTOR];
      float MinJointAng[NUMMOTOR];
      */
      std::vector<double> JointAng;
      std::vector<double> PreJointAng;
      std::vector<double> JointVel;
      std::vector<double> PreJointVel;
      
      std::vector<double> JointAng_Deg;
      
      std::vector<double> m_MuscleReflex;
      std::vector<double> m_ScaMuscleReflex;
      

//      time_t StartTime;
//      time_t NowTime;

            timeval StartTime;
            timeval NowTime;

      double dTimeDiff;
      


      std::vector<double> ScaExtforce;
      std::vector<double> PreScaExtforce;
      
      std::vector<double> ScaMuscleAct;
      std::vector<double> PreScaMuscleAct;
      std::vector<double> bStance;
      std::vector<double> m_reflex_muscle; //  muscle motor command (19 motors)
      std::vector<double> m_Prereflex; 
      std::vector<double> Raw_R_fs_old;
      std::vector<double> Raw_R_fs;
      std::vector<double> Raw_L_fs_old;
      std::vector<double> Raw_L_fs;

      std::vector<double> Exp_L_fs;
      std::vector<double> Exp_R_fs;
      std::vector<double> Exp_L_fs_old;
      std::vector<double> Exp_R_fs_old;
      std::vector<double> Diff_L_fs;
      std::vector<double> Diff_R_fs;
      std::vector<double> AveDiff_L_fs;
      std::vector<double> AveDiff_R_fs;
      std::vector<double> OldAveDiff_L_fs;
      std::vector<double> OldAveDiff_R_fs;

      std::vector<double> acc_Diff_R_fs;

      std::vector<double> acc_Diff_L_fs;

      std::vector<double> old_Diff_L_fs;
      std::vector<double> deri__Diff_L_fs;

      std::vector<double> SideForce;

      std::vector<double> SideForce_old;

      std::vector<double> Stiff_L;
      std::vector<double> Stiff_R;

      std::vector<double> Stiff_L_Fast;
      std::vector<double> Stiff_L_Slow;
      std::vector<double> Stiff_R_Fast;
      std::vector<double> Stiff_R_Slow;

      std::vector<double> FTiStiff_L;
      std::vector<double> FTiStiff_R;

      std::vector<double> FTiStiff_L_Fast;
      std::vector<double> FTiStiff_L_Slow;
      std::vector<double> FTiStiff_R_Fast;
      std::vector<double> FTiStiff_R_Slow;

      double dLDRLR0;
      double dLDRLR1;
      double dLDRLR2;
      double dLDRLL0;
      double dLDRLL1;
      double dLDRLL2;



      double dExErrorR0;
      double dExErrorR1;
      double dExErrorR2;
      double dExErrorL0;
      double dExErrorL1;
      double dExErrorL2;

      double dSlowR0;
      double dSlowR1;
      double dSlowR2;
      double dSlowL0;
      double dSlowL1;
      double dSlowL2;

      double dFastR0;
      double dFastR1;
      double dFastR2;
      double dFastL0;
      double dFastL1;
      double dFastL2;

      double dFTiSlowR0;
      double dFTiSlowR1;
      double dFTiSlowR2;
      double dFTiSlowL0;
      double dFTiSlowL1;
      double dFTiSlowL2;

      double dFTiFastR0;
      double dFTiFastR1;
      double dFTiFastR2;
      double dFTiFastL0;
      double dFTiFastL1;
      double dFTiFastL2;

      double dGainR0;
      double dGainR1;
      double dGainR2;
      double dGainL0;
      double dGainL1;
      double dGainL2;

      double dAutoCorR0;
      double dAutoCorR1;
      double dAutoCorR2;
      double dAutoCorL0;
      double dAutoCorL1;
      double dAutoCorL2;

      int TouchR0;
      int TouchR1;
      int TouchR2;
      int TouchL0;
      int TouchL1;
      int TouchL2;

      double dAveErrorR0;
      double dAveErrorR1;
      double dAveErrorR2;
      double dAveErrorL0;
      double dAveErrorL1;
      double dAveErrorL2;

      double dPreAveErrorR0;
      double dPreAveErrorR1;
      double dPreAveErrorR2;
      double dPreAveErrorL0;
      double dPreAveErrorL1;
      double dPreAveErrorL2;



      int iChangeStiff;
      int iPeriodCount;
      double dPreX;
      double dXDis;
      double dMaxZ;
      double dMinZ;
      double dDiffZ;
      double dPreY;
      double dYDis;
      int iFileOpen;
      int iGetInitialZ;
      
      double dRawX;
      double dRawY;
      double dRawZ;
      
      double dDis;
      double dCRMax;
      double dCRMin;
      double dFRMax;
      double dFRMin;
      double dSWFTiOffset;
      double dSWFTiFac;
      
      ofstream FileX;
      ofstream FileY;
      ofstream FileMaxZ;
      ofstream FileMinZ;
      ofstream FileDiffZ;
      ofstream FileK1;
      ofstream FileK2;
      
      ofstream FileMaxCR;
      ofstream FileMinCR;
      ofstream FileMaxFR;
      ofstream FileMinFR;
      ofstream FileMinDis;
      
      
      double AngleToNeuron(double Input, double MaxAngle, double MinAngle, double MaxNeu, double MinNeu)
      {
        double Val;
        Val = MinNeu + (((Input - MinAngle) * (MaxNeu - MinNeu)) / (MaxAngle - MinAngle));
        return Val;
      }
      
      double VirtualMusclesAcce1(double Ang, double DAng, double t, double Exforce, 
          double MuscleAct, double DisVe, double K, double D, double Leng, double Theta1, double MusclActFac)
      {
        double ExVec,ExforceTor,PasiForceTor,ActiForceTor,NeedAngle;
        
        ExVec = (Leng + Radius);//*cos(Ang) + DisVe;
        
        ExforceTor = Exforce * sin(Ang) * ExVec;
        PasiForceTor = (2 * ((D * DAng) + (K * Ang)) * Radius) * Radius;
        ActiForceTor = MuscleAct * Radius *MusclActFac;
        
        NeedAngle = (ExforceTor + ActiForceTor - PasiForceTor)/(Mass * Radius * Radius / 2.0);
        
        return NeedAngle;
        
      }
      
      double VirtualMusclesAcceN(double Ang, double DAng, double t, double Exforce, 
            double MuscleAct, double DisVe, double K, double D, double Leng, double Theta1, double MusclActFac)
        {
          double ExVec,ExforceTor,PasiForceTor,ActiForceTor,NeedAngle;
          
          ExVec = (Leng + Radius) * cos(Ang) + DisVe;
          
          ExforceTor = Exforce * ExVec;//Exforce * cos(Theta1) * ExVec;
          PasiForceTor = (2 * ((D * DAng) + (K * Ang)) * Radius) * Radius;
          ActiForceTor = MuscleAct * Radius * MusclActFac;
          
          NeedAngle = (ExforceTor + ActiForceTor - PasiForceTor)/(Mass * Radius * Radius / 2.0);
          
          return NeedAngle;
          
        }
      
      MusclePara RungeKutta41(double x, double v, double dt, double PreExforce, double Exforce, 
          double PreMuscleAct, double MuscleAct, double DisVec, double K, double D, double Leng, double Theta1, double MusclActFac)
      {
        double x1,v1,a1,x2,v2,a2,x3,v3,a3,x4,v4,a4,xf,vf;
        
        x1 = x;
        v1 = v;
        a1 = VirtualMusclesAcce1(x1, v1, dt, PreExforce, PreMuscleAct, DisVec, K, D, Leng, Theta1, MusclActFac);
        
        x2 = x + (0.5 * v1 * dt);
        v2 = v + (0.5 * a1 * dt);
        a2 = VirtualMusclesAcce1(x2, v2, dt/2.0, (PreExforce + Exforce)/2.0, (PreMuscleAct + MuscleAct)/2.0, DisVec, K, D, Leng,Theta1, MusclActFac);
        
        x3 = x + (0.5 * v2 * dt);
        v3 = v + (0.5 * a2 * dt);
        a3 = VirtualMusclesAcce1(x3, v3, dt/2.0, (PreExforce + Exforce)/2.0, (PreMuscleAct + MuscleAct)/2.0, DisVec, K, D, Leng,Theta1, MusclActFac);
        
        x4 = x + (v3 * dt);
        v4 = v + (a3 * dt);
        a4 = VirtualMusclesAcce1(x4, v4, dt, Exforce, MuscleAct, DisVec, K, D, Leng,Theta1, MusclActFac);
        
        xf = x + (dt/6.0) * (v1 + (2 * v2) + (2 * v3) + v4);
        vf = v + (dt/6.0) * (a1 + (2 * a2) + (2 * a3) + a4);
        
        MusclePara MusPara1 = {xf, vf};
        
        return MusPara1;
        
      }
      
      MusclePara RungeKutta42(double x, double v, double dt, double PreExforce, double Exforce, 
            double PreMuscleAct, double MuscleAct, double DisVec, double K, double D, double Leng, double Theta1, double MusclActFac)
        {
          double x1,v1,a1,x2,v2,a2,x3,v3,a3,x4,v4,a4,xf,vf;
          
          x1 = x;
          v1 = v;
          a1 = VirtualMusclesAcceN(x1, v1, dt, PreExforce, PreMuscleAct, DisVec, K, D, Leng, Theta1, MusclActFac);
          
          x2 = x + (0.5 * v1 * dt);
          v2 = v + (0.5 * a1 * dt);
          a2 = VirtualMusclesAcceN(x2, v2, dt/2.0, (PreExforce + Exforce)/2.0, (PreMuscleAct + MuscleAct)/2.0, DisVec, K, D, Leng,Theta1, MusclActFac);
          
          x3 = x + (0.5 * v2 * dt);
          v3 = v + (0.5 * a2 * dt);
          a3 = VirtualMusclesAcceN(x3, v3, dt/2.0, (PreExforce + Exforce)/2.0, (PreMuscleAct + MuscleAct)/2.0, DisVec, K, D, Leng,Theta1, MusclActFac);
          
          x4 = x + (v3 * dt);
          v4 = v + (a3 * dt);
          a4 = VirtualMusclesAcceN(x4, v4, dt, Exforce, MuscleAct, DisVec, K, D, Leng,Theta1, MusclActFac);
          
          xf = x + (dt/6.0) * (v1 + (2 * v2) + (2 * v3) + v4);
          vf = v + (dt/6.0) * (a1 + (2 * a2) + (2 * a3) + a4);
          
          MusclePara MusPara2 = {xf, vf};
          
          
          return MusPara2;
          
        }
      
      double FCProcessing(double dCuVal, double dPreVal)
      {
        return (ALPHA * dCuVal + (1-ALPHA)*dPreVal);
      }

      double LMSLearning(double AveDiffFC, int JointID, double JointStiff)
      {
        double dReStiff;
        //if (AveDiffFC > 0)
     //{
          //dAveErrorR0 =  AveDiff_R_fs.at(0);

          if ((AveDiffFC > FCERROR) and (JointStiff < MAXCTRSTIFF))
          {
            dReStiff = ((0.1*JointStiff + 1.5*CTRSTIFFSETP * AveDiffFC) + (0.7*JointStiff + 1.2*CTRSTIFFSETP * AveDiffFC));

                //JointStiff + CTRSTIFFSETP * AveDiffFC;

            switch(JointID)
            {
              case CR0_m:
                std::cout<<"The CTr stiffness of R0 is"<<dReStiff<<"\n";
                break;
              case CR1_m:
                std::cout<<"The CTr stiffness of R1 is"<<dReStiff<<"\n";
                break;
              case CR2_m:
                std::cout<<"The CTr stiffness of R2 is"<<dReStiff<<"\n";
                break;
              case CL0_m:
                std::cout<<"The CTr stiffness of L0 is"<<dReStiff<<"\n";
                break;
              case CL1_m:
                std::cout<<"The CTr stiffness of L1 is"<<dReStiff<<"\n";
                break;
              case CL2_m:
                std::cout<<"The CTr stiffness of L2 is"<<dReStiff<<"\n";
                break;

            }

          }
          else
          {
            dReStiff = JointStiff;

          }

     //}
        return dReStiff;



      }

//      double RLSLearning(double RawFC, double dJointStiff, double ExpFC, double &Gv, double &Ac)
//      {
//        double error = RawFC - ExpFC;//(RawFC - ExpFC);
//
//        Gv = (Ac * RawFC)/(RLSLAMDA + (RawFC * RawFC * Ac));
//        Ac = (Ac - (Gv *RawFC*Ac))/(RLSLAMDA);
//
////                Gv = (Ac * ExpFC)/(RLSLAMDA + (ExpFC * ExpFC * Ac));
////                Ac = (Ac - (Gv *ExpFC*Ac))/(RLSLAMDA);
//
////                double error = ExpFC - RawFC;//(RawFC - ExpFC);
////
////                Gv = (Ac * error)/(RLSLAMDA + (error * error * Ac));
////                Ac = (Ac - (Gv *error*Ac))/(RLSLAMDA);
//
//        return (dJointStiff + Gv * error);
//      }

      double RLSLearning(double RawFC, double dJointStiff, double ExpFC, double &Gv, double &Ac)
      {
        double error ; //= RawFC - ExpFC;//(RawFC - ExpFC);

        double stiff_temp = 0.0;

       // if (RawFC = 0.0) RawFC = 0.01;

        Gv =  ((1/RLSLAMDA)*(Ac*RawFC))/(1+(1/RLSLAMDA)*RawFC*Ac*RawFC);

        error = -ExpFC + RawFC;

        dJointStiff = dJointStiff + error*Gv;

        Ac = (1/RLSLAMDA)*(Ac - Gv*RawFC*Ac);

//                Gv = (Ac * ExpFC)/(RLSLAMDA + (ExpFC * ExpFC * Ac));
//                Ac = (Ac - (Gv *ExpFC*Ac))/(RLSLAMDA);

//                double error = ExpFC - RawFC;//(RawFC - ExpFC);
//
//                Gv = (Ac * error)/(RLSLAMDA + (error * error * Ac));
//                Ac = (Ac - (Gv *error*Ac))/(RLSLAMDA);

        return (dJointStiff);
      }


      double ExRLSLearning(double RawFC, double dJointStiff, double ExpFC, double &Gv, double &Ac, double &ExError)
      {
        double error = RawFC - ExpFC + ExError;//(RawFC - ExpFC);

        Gv = (Ac * RawFC)/(RLSLAMDA + (RawFC * RawFC * Ac));
        Ac = (Ac - (Gv *RawFC*Ac))/(RLSLAMDA);

//                Gv = (Ac * ExpFC)/(RLSLAMDA + (ExpFC * ExpFC * Ac));
//                Ac = (Ac - (Gv *ExpFC*Ac))/(RLSLAMDA);

//                double error = RawFC - ExpFC;//(RawFC - ExpFC);
//
//                Gv = (Ac * error)/(RLSLAMDA + (error * error * Ac));
//                Ac = (Ac - (Gv *error*Ac))/(RLSLAMDA);
        ExError = error;
        return (dJointStiff + Gv * error);
      }

      double DualRateLearning(double RawFC, double dJointStiff, double ExpFC)// one direction
      {

        double dReStiff;
        double error =  ExpFC - RawFC;
               //if (AveDiffFC > 0)
            //{
                 //dAveErrorR0 =  AveDiff_R_fs.at(0);

                 if ((error > DRLERROR)) //and (dJointStiff < MAXCTRSTIFF))
                 {
                   dReStiff = ((0.15*dJointStiff + 1.5*DRLRATE * error) + (0.7*dJointStiff + 1.2*DRLRATE * error));
                       //((0.15*dJointStiff + 1.5*DRLRATE * error) + (0.7*dJointStiff + 1.2*DRLRATE * error));
                   //0.5*dJointStiff + DRLFRATE * error;
                 }
                 else
                 {
                   dReStiff = dJointStiff;

                 }

            //}
               return dReStiff;



      }

      double FastLearning(double RawFC, double dJointStiff, double ExpFC)// one direction
        {

          double dReStiff;
          double error =  ExpFC - RawFC;
                 //if (AveDiffFC > 0)
              //{
                   //dAveErrorR0 =  AveDiff_R_fs.at(0);

                   if ((error > DRLERROR)) //and (dJointStiff < MAXCTRSTIFF))
                   {
                     if(TDRLEQU == 0)
                     {
                     dReStiff = ((0.15*dJointStiff + 1.5*DRLRATE * error));
                         //((0.15*dJointStiff + 1.5*DRLRATE * error) + (0.7*dJointStiff + 1.2*DRLRATE * error));
                     //0.5*dJointStiff + DRLFRATE * error;
                     }

                     if(TDRLEQU == 4)
                     {
                       dReStiff = (-0.35*dJointStiff + 0.2*DRLRATE * error);//(-0.2*(dJointStiff) + 0.1*DRLRATE * error);

                     }

                     if(TDRLEQU == 5)
                     {
                       dReStiff = (0.59*dJointStiff + 0.21*DRLRATE * error);

                     }

                   }
                   else
                   {
                     dReStiff = dJointStiff;

                   }

              //}
                 return dReStiff;



        }
      double SlowLearning(double RawFC, double dJointStiff, double ExpFC)// one direction
             {

               double dReStiff;
               double error =  ExpFC - RawFC;
                      //if (AveDiffFC > 0)
                   //{
                        //dAveErrorR0 =  AveDiff_R_fs.at(0);

                        if ((error > DRLERROR)) //and (dJointStiff < MAXCTRSTIFF))
                        {
                          if (TDRLEQU == 0)
                          {
                            dReStiff = (0.7*dJointStiff + 1.2*DRLRATE * error);
                                                          //((0.15*dJointStiff + 1.5*DRLRATE * error) + (0.7*dJointStiff + 1.2*DRLRATE * error));
                                                      //0.5*dJointStiff + DRLFRATE * error;

                          }

                          if(TDRLEQU == 4)
                          {
                            dReStiff = (-0.2*(dJointStiff) + 0.1*DRLRATE * error);

                          }

                          if(TDRLEQU == 5)
                          {
                            dReStiff = (0.992*dJointStiff + 0.02*DRLRATE * error);

                          }

                        }
                        else
                        {
                          dReStiff = dJointStiff;

                        }

                   //}
                      return dReStiff;



             }

//      double DualRateLearning(double RawFC, double dJointStiff, double ExpFC)// one direction
//      {
//
//        double dReStiff;
//        double error =  ExpFC - RawFC;
//               //if (AveDiffFC > 0)
//            //{
//                 //dAveErrorR0 =  AveDiff_R_fs.at(0);
//
//                 if ((error > DRLERROR) and (dJointStiff < MAXCTRSTIFF))
//                 {
//                   dReStiff = ((0.15*dJointStiff + 1.5*DRLRATE * error) + (0.7*dJointStiff + 1.2*DRLRATE * error));
//                       //((0.15*dJointStiff + 1.5*DRLRATE * error) + (0.7*dJointStiff + 1.2*DRLRATE * error));
//                   //0.5*dJointStiff + DRLFRATE * error;
//                 }
//                 else
//                 {
//                   dReStiff = dJointStiff;
//
//                 }
//
//            //}
//               return dReStiff;
//
//
//
//      }

      double TDualRateLearning(double RawFC, double dJointStiff, double ExpFC)// Two direction
      {

             double dReStiff;
             double error = ExpFC-RawFC;
                    //if (AveDiffFC > 0)
                 //{
                      //dAveErrorR0 =  AveDiff_R_fs.at(0);

                      if ((error > DRLERROR)) //and (dJointStiff < MAXCTRSTIFF))
                      {
                        if (TDRLEQU == 2)
                        {
                          dReStiff = ((0.15*dJointStiff + 1.5*DRLRATE * error) + (0.7*(0.9+dJointStiff) + 1.2*DRLRATE * error));

                        }

                        if(TDRLEQU == 3)
                        {
                          dReStiff = ((0.15*dJointStiff - 1.5*DRLRATE * error) + (0.7*(1.8+dJointStiff) - 1.2*DRLRATE * error));

                        }


                        //


                        //dual rate and direction learners

                        if (TDRLEQU == 0)
                        {
                          dReStiff = ((0.15*dJointStiff + 1.5*DRLRATE * error) + (0.7*(12-dJointStiff) - 1.2*DRLRATE * error));
                          //original function

                        }

                        if (TDRLEQU == 1)
                        {
                          dReStiff = ((0.15*(dJointStiff+5.3) + 1.5*DRLRATE * error) + (0.7*(dJointStiff) - 1.2*DRLRATE * error));

                        }

                        if (TDRLEQU == 4)
                        {
                          dReStiff = 12 * 0.7 + ((-0.35*dJointStiff + 0.2*DRLRATE * error) + (-0.2*(dJointStiff) + 0.1*DRLRATE * error));
                          //original function

                        }




                        //
                      }
                      else
                      {
                        dReStiff = dJointStiff;

                      }

                 //}
                    return dReStiff;



      }

      double LTDualRateLearning(double RawFC, double dJointStiff, double ExpFC, double &dValue)// Two direction
           {

                  double dReStiff;
                  double error = ExpFC-RawFC;
                         //if (AveDiffFC > 0)
                      //{
                           //dAveErrorR0 =  AveDiff_R_fs.at(0);
//
//                  if (abs(error) > 0.1)
//                  {
//                    dValue = dValue + abs(error) * 0.015;
//
//                  }

                           if ((error > DRLERROR)) //and (dJointStiff < MAXCTRSTIFF))
                           {
                              if(TDRLEQU == 0)
                              {
                             //dReStiff = ((0.15*dJointStiff + 1.5*DRLRATE * error) + (0.7*(12-dJointStiff) + 1.2*DRLRATE * error));


                             //dual rate and direction learners
                             dReStiff = ((0.15*dJointStiff + 1.5*DRLRATE * error) + (0.7*(dValue-dJointStiff) - 1.2*DRLRATE * error));
                             //dReStiff = ((0.15*(dJointStiff+5.3) + 1.5*DRLRATE * error) + (0.7*(dJointStiff) - 1.2*DRLRATE * error));
                              }

                              if(TDRLEQU == 4)
                              {
                                dReStiff = dValue * 0.7 + ((-0.35*dJointStiff + 0.2*DRLRATE * error) + (-0.2*(dJointStiff) + 0.1*DRLRATE * error));


                              }

                           }
                           else
                           {
                             dReStiff = dJointStiff;

                           }

                      //}
                         return dReStiff;



           }


      double SDualRateLearning(double RawFC, double dJointStiff, double ExpFC, double &dSlow, double &dFast)// Two direction
             {

                    double dReStiff;
                    double error = ExpFC-RawFC;

                           //if (AveDiffFC > 0)
                        //{
                             //dAveErrorR0 =  AveDiff_R_fs.at(0);
  //
  //                  if (abs(error) > 0.1)
  //                  {
  //                    dValue = dValue + abs(error) * 0.015;
  //
  //                  }

                             if ((error > DRLERROR)) //and (dJointStiff < MAXCTRSTIFF))
                             {

                               //dReStiff = ((0.15*dJointStiff + 1.5*DRLRATE * error) + (0.7*(12-dJointStiff) + 1.2*DRLRATE * error));


                               //dual rate and direction learners
//                               dFast = (0.35*dFast + 1.5*DRLRATE * error); // use 0.45 NOT 0.35, the convergence will blow out
//                               dSlow = (0.9*dSlow + 1.2*DRLRATE * error); // use 1.0 NOT 0.9, the convergence will blow out


                               if(TDRLEQU == 4)
                               {

                               dFast = (-0.35*dFast + 0.2*DRLRATE * error);
                               dSlow = (-0.2*dSlow + 0.1*DRLRATE * error);

                               dReStiff = dFast + dSlow;
                               //dReStiff = ((0.15*(dJointStiff+5.3) + 1.5*DRLRATE * error) + (0.7*(dJointStiff) - 1.2*DRLRATE * error));
                               }

                               if (TDRLEQU == 5)
                               {
                                 if(SPONRECOVE ==1)
                                 {
                                 if (RawFC < 0.01)
                                 {
                                   error = 0.0;

                                 }
                                 }

                                 dFast = (0.59*dFast + 0.21*DRLRATE * error);
                                 dSlow = (0.992*dSlow + 0.02*DRLRATE * error);

                                 dReStiff = dFast + dSlow + 5.7;


                               }
                             }
                             else
                             {
                               dReStiff = dJointStiff;

                             }

                        //}
                           return dReStiff;



             }

      double ParaSDualRateLearning(double RawFC, double dJointStiff, double ExpFC, double &dSlow, double &dFast,double &dPValue)// Two direction
                  {

                         double dReStiff;
                         double error = ExpFC-RawFC;
                                //if (AveDiffFC > 0)
                             //{
                                  //dAveErrorR0 =  AveDiff_R_fs.at(0);
       //
       //                  if (abs(error) > 0.1)
       //                  {
       //                    dValue = dValue + abs(error) * 0.015;
       //
       //                  }

                                  if ((error > DRLERROR)) //and (dJointStiff < MAXCTRSTIFF))
                                  {

                                    //dReStiff = ((0.15*dJointStiff + 1.5*DRLRATE * error) + (0.7*(12-dJointStiff) + 1.2*DRLRATE * error));


                                    //dual rate and direction learners
     //                               dFast = (0.35*dFast + 1.5*DRLRATE * error); // use 0.45 NOT 0.35, the convergence will blow out
     //                               dSlow = (0.9*dSlow + 1.2*DRLRATE * error); // use 1.0 NOT 0.9, the convergence will blow out


                                    if(TDRLEQU == 4)
                                    {

                                    dFast = (-0.35*dFast + 0.2*DRLRATE * error);
                                    dSlow = (-0.2*dSlow + 0.1*DRLRATE * error);

                                    dReStiff = dFast + dSlow;
                                    //dReStiff = ((0.15*(dJointStiff+5.3) + 1.5*DRLRATE * error) + (0.7*(dJointStiff) - 1.2*DRLRATE * error));
                                    }

                                    if (TDRLEQU == 5)
                                    {

                                      dFast = (0.59*dFast + 0.21*DRLRATE * error);
                                      dSlow = (0.992*dSlow + 0.02*DRLRATE * error);

                                      dReStiff = dFast + dSlow + dPValue;
                                      //dReStiff = dFast + dSlow + dReStiff;

                                    }
                                  }
                                  else
                                  {
                                    dReStiff = dJointStiff;

                                  }

                             //}
                                return dReStiff;



                  }


      double TSSDualRateLearning(double RawFC, double dJointStiff, double ExpFC, double &dSlow, double &dFast, double dBaseVal)// Two direction
                  {

                         double dReStiff;
                         double error = ExpFC-RawFC;
                                //if (AveDiffFC > 0)
                             //{
                                  //dAveErrorR0 =  AveDiff_R_fs.at(0);
       //
       //                  if (abs(error) > 0.1)
       //                  {
       //                    dValue = dValue + abs(error) * 0.015;
       //
       //                  }

                                  if ((error > DRLERROR)) //and (dJointStiff < MAXCTRSTIFF))
                                  {

                                    //dReStiff = ((0.15*dJointStiff + 1.5*DRLRATE * error) + (0.7*(12-dJointStiff) + 1.2*DRLRATE * error));


                                    //dual rate and direction learners
     //                               dFast = (0.35*dFast + 1.5*DRLRATE * error); // use 0.45 NOT 0.35, the convergence will blow out
     //                               dSlow = (0.9*dSlow + 1.2*DRLRATE * error); // use 1.0 NOT 0.9, the convergence will blow out


                                    if(TDRLEQU == 4)
                                    {

                                    dFast = (-0.35*dFast + 0.2*DRLRATE * error);
                                    dSlow = (-0.2*dSlow + 0.1*DRLRATE * error);

                                    dReStiff = dFast + dSlow;
                                    //dReStiff = ((0.15*(dJointStiff+5.3) + 1.5*DRLRATE * error) + (0.7*(dJointStiff) - 1.2*DRLRATE * error));
                                    }

                                    if (TDRLEQU == 5)
                                    {
                                      /*

                                      dFast = (0.59*dFast + 0.21*DRLRATE * error);
                                      dSlow = (0.992*dSlow + 0.02*DRLRATE * error);

                                      // all values are from reza`s paper.
                                      */


                                      //float incresefactor = 3;
                                      dFast = (0.59*dFast + dBaseVal*0.21 * error);
                                      dSlow = (0.992*dSlow + dBaseVal*0.02 * error);

                                      dReStiff = dFast + dSlow;//dBaseVal*(1.5*error + 1.5*error*error);//dFast + dSlow + dBaseVal*(1 + error);
                                      //dReStiff = dFast + dSlow + error *14.0;

                                    }
                                  }
                                  else
                                  {
                                    dReStiff = dJointStiff;

                                  }

                             //}
                                return dReStiff;



                  }

      double Scaling(double Input, double MaxIn, double MinIn, double MaxOut, double MinOut)
      {
        
//        double dReVal = (MinOut + ((Input - MinIn)/(MaxIn - MinIn)) * (MaxOut - MinOut));
//
//        if (dReVal > 1)
//        {
//          dReVal = 1;
//        }
//
//        if (dReVal < -1)
//        {
//          dReVal = -1;
//        }

        return (MinOut + ((Input - MinIn)/(MaxIn - MinIn)) * (MaxOut - MinOut));


//        return dReVal;

      }
      

      
      
//      MuscleOut VMuscles(MuscleIn Mu, JPara FJ, JPara CJ)
//      {
//        
//        MuscleOut Out;
//        
//            if (Mu.Neu < Mu.PreNeu)// && (Raw_R_fs.at(1) > FSERR)
//            {
//              
//              MusclePara ParaR11 = RungeKutta41(FJ.PrePos, FJ.PreVel, Mu.SampleTime , Mu.PreFc, 
//                  Mu.Fc, FJ.PreNeu, FJ.Neu, 0, FJ.K, FJ.D, FJ.MusLeng, 0, FJ.MusFac);
//              
//              Out.FJPos = ParaR11.Pos;
//              Out.FJNeu = Scaling(Out.FJPos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
//              Out.FJVel = ParaR11.Vel;
//                          
//              double DisVe = (FJ.MusLeng + Mu.Radius) * sin(Out.FJPos);
//              MusclePara ParaR12 = RungeKutta42(CJ.PrePos, CJ.PreVel, Mu.SampleTime , Mu.PreFc, 
//                  Mu.Fc, CJ.PreNeu, CJ.Neu, DisVe, CJ.K, CJ.D, CJ.MusLeng, Out.FJPos, CJ.MusFac);
//              
//              Out.CJPos = ParaR12.Pos;
//              Out.CJNeu = Scaling(Out.CJPos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
//              Out.CJVel = ParaR12.Vel;
//
//              
//            }
//            else
//            {
//              
//
//              Out.FJNeu = FJ.Neu;
//              Out.FJPos = Scaling(Out.FJNeu, FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
//              Out.FJVel = 0;
//              Out.CJNeu = CJ.Neu;
//              Out.CJPos = Scaling(Out.CJNeu, CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
//              Out.CJVel = 0;
//              
//            }
//            
//        return Out;
//      }
//      
      
      

	//---Start Define functions---//
	NeuralLocomotionControlAdaptiveClimbing();
	~NeuralLocomotionControlAdaptiveClimbing();

	double sigmoid(double num)
	{
	return 1./(1.+exp(-num));
	}


	std::vector<double> step_nlc(const std::vector<double> in0 /*from neural preprocessing [-1 (stance), +1 (swing)]*/, const std::vector<double> in1 /*from neural learning*/);//, bool Footinhibition=false);
	// if several sensor values (like preprocessed or output of memory and learning are used, extend parameters)
	// std::vector<double> step_nlc(const std::vector<double> in0, const std::vector<double> in1, const std::vector<double> in2);

	//---End  Define functions---//



	//---Start Save files---//
	ofstream outFilenlc1;


	//---End Save files---//


	//---Start Define vector----//

	//---Input neurons
	std::vector<double> input;

	//---CPG
	std::vector<double> cpg_activity; 					//CPG neural activities
	std::vector<double> cpg_output;						//CPG neural outputs
	std::vector< std::vector<double> > cpg_w; 			//CPG neural weights
	double cpg_bias;									//CPG bias
	double Control_input;

	//---pCPG
	std::vector<double> pcpg_output;					//pCPG neural outputs
	std::vector<double> pcpg_step;						//step function
	std::vector<double> set;			   		        //step function
	std::vector<double> setold;			   		    	//step function
	std::vector<double> diffset;			   		    //step function

	std::vector<double> countup; 						//counter
	std::vector<double> countupold;						//counter
	std::vector<double> countdown;						//counter
	std::vector<double> countdownold; 					//counter

	std::vector<double> deltaxup;						//delta
	std::vector<double> deltaxdown;						//delta

	std::vector<double> xup;							//delta
	std::vector<double> xdown;							//delta

	std::vector<double> yup;							//delta
	std::vector<double> ydown;							//delta


	//---PSN
	std::vector<double> psn_activity; 					//PSN neural activities
	std::vector<double> psn_output;						//PSN neural outputs
	std::vector< std::vector<double> > psn_w;			//PSN neural weights
	std::vector<double> psn_bias;						//PSN bias

	//---VRN
	std::vector<double> vrn_activity; 					//VRN neural activities
	std::vector<double> vrn_output;						//VRN neural outputs
	std::vector< std::vector<double> > vrn_w; 			//VRN neural weights
	double vrn_bias;

	//---Interconnections
	std::vector< std::vector<double> > psn_pcpg_w; 		//PSN neural weights
	std::vector< std::vector<double> > vrn_psn_w; 		//VRN neural weights

	//---Interconnections--input2 to PSN
	std::vector< std::vector<double> > psn_input2_w;    //PSN neural weights
	double vrn_input3_w;
	double vrn_input4_w;

	//---Motor neurons
	std::vector<double> tr_activity; 				    //motor neural activities
	std::vector<double> tl_activity;					//motor neural activities
	std::vector<double> cr_activity; 				    //motor neural activities
	std::vector<double> cl_activity;					//motor neural activities
	std::vector<double> fr_activity; 				    //motor neural activities
	std::vector<double> fl_activity;					//motor neural activities
	std::vector<double> bj_activity;					//motor neural activities

	std::vector<double> tr_output; 				    	//motor neural outputs
	std::vector<double> tl_output;						//motor neural outputs
	std::vector<double> cr_output; 				    	//motor neural outputs
	std::vector<double> cl_output;						//motor neural outputs
	std::vector<double> fr_output; 				   	 	//motor neural outputs
	std::vector<double> fl_output;						//motor neural outputs
	std::vector<double> bj_output;						//motor neural outputs

	std::vector<double> cr_outputold; 					//motor neural outputs_old
	std::vector<double> cl_outputold;					//motor neural outputs_old

	std::vector<double> diffcr_output; 					//diff motor neural outputs
	std::vector<double> diffcl_output;					//diff motor neural outputs

	std::vector<double> postcr;							//postCL motor neural outputs
	std::vector<double> postcl;							//postCR motor neural outputs
	std::vector<double> postcrold;						//postCL motor neural outputs
	std::vector<double> postclold;						//postCR motor neural outputs
	double threshold_c;

	std::vector<double> buffer_t;						//delay buffer vector thorical
//	std::vector<double> buffer_c;						//delay buffer vector coxa
//	std::vector<double> buffer_f;						//delay buffer vector fibia
	std::vector<double> buffer_tr;
	std::vector<double>  buffer_tl;
	std::vector<double>  buffer_c;
	std::vector<double>  buffer_f;
	std::vector<double>  buffer_fm;

	std::vector<double> m_pre;							//pre motor outputs (19 motors)
	std::vector<double> m_preold;           //pre motor outputs old (19 motors)
	std::vector<double> m_prescale;         //pre motor outputs (19 motors)
	std::vector<double> m_reflex;						//reflex motor outputs  (19 motors)
	std::vector<double> m;								  //motor outputs as neural activation (19 motors)
	std::vector<double> m_deg;							//motor outputs in deg (19 motors)

	//---Reflex motor neurons
	std::vector<double> fmodel_cmr_activity;			//coxa motor right neural activities
	std::vector<double> fmodel_cmr_output; 				//coxa motor right neural outputs
	std::vector<double> fmodel_cmr_output_old;
	std::vector<double> fmodel_cmr_error;			    //error coxa motor right and foot right
	std::vector<double> fmodel_cmr_errorW;
	std::vector<double> fmodel_cmr_outputfinal; 		//coxa motor right neural outputs
	std::vector<double> low_pass_fmodel_cmr_error_old;
	std::vector<double> low_pass_fmodel_cmr_error;

	std::vector<double> fmodel_cml_activity;			//coxa motor left neural activities
	std::vector<double> fmodel_cml_output; 				//coxa motor left neural outputs
	std::vector<double> fmodel_cml_output_old;
	std::vector<double> fmodel_cml_error;			    //error coxa motor left and foot left
	std::vector<double> fmodel_cml_errorW;
	std::vector<double> fmodel_cml_outputfinal; 		//coxa motor right neural outputs
	std::vector<double> low_pass_fmodel_cml_error_old;
	std::vector<double> low_pass_fmodel_cml_error;

	//---Reflex foot sensors
	std::vector<double> reflex_R_fs;
	std::vector<double> reflex_L_fs;
	std::vector<double> reflex_R_fs_old;//KOH add after the GIT version
	std::vector<double> reflex_L_fs_old;//KOH add after the GIT version
	std::vector<double> countup_reflex_R_fs;//KOH add after the GIT version
	std::vector<double> countdown_reflex_R_fs;//KOH add after the GIT version
	std::vector<double> countup_reflex_R_fs2;//KOH add after the GIT version
	int counter_refelx_R_fs;//KOH add after the GIT version
	int counter_refelx_L_fs;//KOH add after the GIT version
	int max_up, max_down;
	double max_fmodel;//KOH add after the GIT version
	std::vector<double>  dervi_reflex_R_fs;
	std::vector<double>  dervi_reflex_L_fs;


	//Learning forward models to expected foot sensors
	std::vector<double> lr_fmodel_cr;					//learning rate
	std::vector<double> fmodel_cmr_w;					//forward model weights
	std::vector<double> fmodel_fmodel_cmr_w;			//forward model recurrent weights
	std::vector<double> fmodel_post_cmr_w;				//forward model postprocessing weights
	std::vector<double> fmodel_cmr_bias;				//forward model biases
	std::vector<double>  acc_cmr_error;					//forward model biases
	std::vector<double>  acc_cmr_error_old;				//forward model biases
	std::vector<double>  deri_acc_cmr_error;			//forward model biases
	std::vector<double>  acc_cmr_error_elev;			//error for elevator reflex
	std::vector<double>  error_cmr_elev;				//error for elevator reflex
	std::vector<double>  acc_cmr_error_posi_neg;		//forward model biases
	std::vector<double>  dervi_fmodel_cmr_output;

	std::vector<double> lr_fmodel_cl;					//learning rate
	std::vector<double> fmodel_cml_w;					//forward model weights
	std::vector<double> fmodel_fmodel_cml_w;			//forward model recurrent weights
	std::vector<double> fmodel_post_cml_w;				//forward model postprocessing weights
	std::vector<double> fmodel_cml_bias;				//forward model biases
	std::vector<double>  acc_cml_error;					//forward model biases
	std::vector<double>  acc_cml_error_old;				//forward model biases
	std::vector<double>  deri_acc_cml_error;			//forward model biases
	std::vector<double>  acc_cml_error_elev;			//error for elevator reflex
	std::vector<double>  error_cml_elev;				//error for elevator reflex
	std::vector<double>  acc_cml_error_posi_neg;		//forward model biases
	std::vector<double>  dervi_fmodel_cml_output;


	std::vector<double> lowpass_cmr_error_activity;		//lowpass neuron activities
	std::vector<double> lowpass_cmr__error_output; 		//lowpass neuron outputs
	std::vector<double> lowpass_cmr_w;					//lowpass weights
	std::vector<double> lowpass_lowpass_cmr_w;			//lowpass recurrent weights
	std::vector<double> lowpass_cmr_bias;				//lowpass biases

	std::vector<double> lowpass_cml_error_activity;		//lowpass neuron activities
	std::vector<double> lowpass_cml__error_output; 		//lowpass neuron outputs
	std::vector<double> lowpass_cml_w;					//lowpass weights
	std::vector<double> lowpass_lowpass_cml_w;			//lowpass recurrent weights
	std::vector<double> lowpass_cml_bias;				//lowpass biases


	std::vector<double> a1_r;							//Emd_2D, learning parameters
	std::vector<double> a2_r;							//Emd_2D, learning parameters
	std::vector<double> a3_r;							//Emd_2D, learning parameters
	std::vector<double> fcn_r; 							//Emd_2D
	std::vector<double> fac_r; 							//Emd_2D
	std::vector<double> normxsq_r;						//Emd_2D
	std::vector<double> pred_r;
	std::vector<double> a1_l;							//Emd_2D, learning parameters
	std::vector<double> a2_l;							//Emd_2D, learning parameters
	std::vector<double> a3_l;							//Emd_2D, learning parameters
	std::vector<double> fcn_l; 							//Emd_2D
	std::vector<double> fac_l; 							//Emd_2D
	std::vector<double> normxsq_l;						//Emd_2D
	std::vector<double> pred_l;
	std::vector<double> delay_CR0;						//Delay signal
	std::vector<double> delay_CR1;						//Delay signal
	std::vector<double> delay_CR2;						//Delay signal
	std::vector<double> delay_CL0;						//Delay signal
	std::vector<double> delay_CL1;						//Delay signal
	std::vector<double> delay_CL2;						//Delay signal
	std::vector<double> m_pre_delay;




	//Motor mapping

	double min_tc; // network output range
	double max_tc;// network output range
	//Adjust
	double min_tc_f_nwalking_deg; //deg **
	double max_tc_f_nwalking_deg; //deg **
	double min_tc_f_nwalking;
	double max_tc_f_nwalking;

	//TC_middle
	//Adjust
	double min_tc_m_nwalking_deg; //deg **
	double max_tc_m_nwalking_deg; //deg **
	double min_tc_m_nwalking;
	double max_tc_m_nwalking;

	//TC_rear
	//Adjust
	double min_tc_r_nwalking_deg; //deg **
	double max_tc_r_nwalking_deg; //deg **
	double min_tc_r_nwalking;
	double max_tc_r_nwalking;

	//CTR joints
	double min_ctr; // network output range
	double max_ctr;// network output range

	std::vector<double>  min_ctr_nwalking_deg;//deg
	std::vector<double>  max_ctr_nwalking_deg;//deg
	std::vector<double>  min_ctr_nwalking;
	std::vector<double>  max_ctr_nwalking;
	std::vector<double>  offset_ctr;

	std::vector<double>  min_ctl_nwalking_deg;//deg
	std::vector<double>  max_ctl_nwalking_deg;//deg
	std::vector<double>  min_ctl_nwalking;
	std::vector<double>  max_ctl_nwalking;
	std::vector<double>  offset_ctl;


	//FTI joints
	double min_fti; // network output range
	double max_fti; // network output range

	std::vector<double>  min_ftir_nwalking_deg;//deg
	std::vector<double>  max_ftir_nwalking_deg;//deg
	std::vector<double>  min_ftir_nwalking;
	std::vector<double>  max_ftir_nwalking;
	std::vector<double>  offset_ftir;

	std::vector<double>  min_ftil_nwalking_deg;//deg
	std::vector<double>  max_ftil_nwalking_deg;//deg
	std::vector<double>  min_ftil_nwalking;
	std::vector<double>  max_ftil_nwalking;
	std::vector<double>  offset_ftil;


	//BJ joint
	double min_bj; // network output range
	double max_bj;// network output range
	double min_bj_fwalking_deg; //deg
	double max_bj_fwalking_deg; //deg
	double min_bj_fwalking;
	double max_bj_fwalking;

    double max_c; // max range
    double max_f; // max range
    double max_c_offset; // Adjust from 45,...185=> 185 (cin= 0.02, wave), 45 (cin= 0.18, tripod)
    double max_f_offset; // Adjust from 45,...185=> 185 (cin= 0.02, wave), 45 (cin= 0.18, tripod)

    //Reading motor signals from Text
    vector<double> m_r0_t;
    vector<double> m_r1_t;
    vector<double> m_r2_t;
    vector<double> m_l0_t;
    vector<double> m_l1_t;
    vector<double> m_l2_t;

    double m_r0_t_old;
    double m_r1_t_old;
    double m_r2_t_old;
    double m_l0_t_old;
    double m_l1_t_old;
    double m_l2_t_old;

    double m_r0_text;
    double m_r1_text;
    double m_r2_text;
    double m_l0_text;
    double m_l1_text;
    double m_l2_text;



    int i_text_loop;
    int ii;
    bool initialized;


    std::vector<double> m_deg_test;

	//---End Define vector----//

private:

	double  h;
	double count1;
	double count2;

	int T1;
	int T2;
	int T1old;
	int T2old;
	int period1;
	int period2;
	double y1;
	double y2;
	std::vector<double> triH1; //triangle output vector
	std::vector<double> triH2;

	int tau;
	int tau_l;
	//int time; //time>

	int option_wiring;
	std::vector<double> counter_cr;
	std::vector<double> counter_cl;

	int global_count;
	int allfoot_off_ground;

	int option_fmodel;
	bool switchon_ED;
	bool switchon_footinhibition;
	bool switchon_reflexes;
	bool switchon_purefootsignal;
	bool switchon_learnweights;
	bool softlanding;
	bool reading_text_testing;
};

#endif /* NEURALLOCOMOTIONCONTROLADAPTIVECLIMBING_H_ */
