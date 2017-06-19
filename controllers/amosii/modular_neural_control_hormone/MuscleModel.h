/*
 * MuscleModel.h
 *
 *  Created on: May 26, 2014
 *      Author: Subhi Shaker Barikhan
 */


#ifndef MUSCLEMODEL_H_
#define MUSCLEMODEL_H_

#include <vector>
#include <cmath>
#include <cstdlib>
#include <ode_robots/amosiisensormotordefinition.h>


//Save files
#include <iostream>
#include <fstream>
#include <string.h>


#define FSSCAL 2.0
//#define ISMUSCLE 1// 0 is multiple CPG with sensory feedback.
#define FTIUP 0.4
#define SampleTime 0.01
#define HipLength 0.115
#define KneeLength 0.075
#define FMusclActFac 0
#define CMusclActFac 1
#define Mass 0.1
#define Radius 0.1
#define CRMAXOUT 1
#define CRMINOUT -1
#define CRMAXANG ((100.0/180.0)*3.14159265358979)//((100.0/180.0)*3.14159265358979)
#define CRMINANG ((-45.0/180.0)*3.14159265358979)//((-30.0/180.0)*3.14159265358979)
#define FRMAXOUT 1
#define FRMINOUT -1
#define FRMAXANG ((55.0/180.0)*3.14159265358979)//((75.0/180.0)*3.14159265358979)
#define FRMINANG ((-70.0/180.0)*3.14159265358979)//((-50.0/180.0)*3.14159265358979)

#define MINCTROUT 0.71
#define MINFTIOUT -1.0
typedef struct{
    double Pos;
    double Vel;
}MusclePara;

using namespace std;



class MuscleModel{
public:

	//---Start Define functions---//
	MuscleModel(int aAmosVersion);
	~MuscleModel();


	//------------------Muscle definitions and functions------------------------//

	std::vector<double> JointAng;
 std::vector<double> PreJointAng;
 std::vector<double> JointVel;
 std::vector<double> PreJointVel;

 std::vector<double> JointAng_Deg;

 std::vector<double> m_MuscleReflex;
 std::vector<double> m_ScaMuscleReflex;


	//      time_t StartTime;
	//      time_t NowTime;

 timeval StartTime;
 timeval NowTime;

double dTimeDiff;

double dSWFTiOffset;
double dSWFTiFac;

     std::vector<double> ScaExtforce;
    std::vector<double> PreScaExtforce;

   std::vector<double> ScaMuscleAct;
    std::vector<double> PreScaMuscleAct;
     std::vector<double> bStance;
     std::vector<double> m_reflex_muscle; //  muscle motor command (19 motors)
     std::vector<double> m_Prereflex;
std::vector<double> Raw_R_fs_old;
std::vector<double> Raw_R_fs;
 std::vector<double> Raw_L_fs_old;
 std::vector<double> Raw_L_fs;

std::vector<double> m_preN;//pre motor outputs (19 motors)
std::vector<double> m_preold;//pre motor outputs old (19 motors)

	double K1[6];
	double K2[6];

	double D1[6];
	double D2[6];







	double VirtualMusclesAcce1(double Ang, double DAng, double t, double Exforce, double MuscleAct, double DisVe, double K, double D, double Leng, double Theta1, double MusclActFac);//step function for FTi joints,
	double VirtualMusclesAcceN(double Ang, double DAng, double t, double Exforce, double MuscleAct, double DisVe, double K, double D, double Leng, double Theta1, double MusclActFac);//step function for CTr // joints, see RungeKutta42(…)
	MusclePara RungeKutta41(double x, double v, double dt, double PreExforce, double Exforce, double PreMuscleAct, double MuscleAct, double DisVec, double K, double D, double Leng, double Theta1, double MusclActFac); // for FTi joints

	MusclePara RungeKutta42(double x, double v, double dt, double PreExforce, double Exforce, double PreMuscleAct, double MuscleAct, double DisVec, double K, double D, double Leng, double Theta1, double MusclActFac); // for CTr joints

	double Scaling(double Input, double MaxIn, double MinIn, double MaxOut, double MinOut);
    std::vector<double> getMuscleReflex(std::vector<double> m_reflex,const std::vector<double> inreflex);



private:



};
#endif /* MUSCLEMODEL_H_ */
