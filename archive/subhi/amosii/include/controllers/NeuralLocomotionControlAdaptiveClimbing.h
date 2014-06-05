/*
 * NeuralLocomotionControlAdaptiveClimbing.h
 *
 *  Created on: May 2, 2011
 *      Author: poramate
 */

#ifndef NEURALLOCOMOTIONCONTROLADAPTIVECLIMBING_H_
#define NEURALLOCOMOTIONCONTROLADAPTIVECLIMBING_H_

#include <vector>
#include <cmath>
#include <cstdlib>
#include <ode_robots/amosiisensormotordefinition.h>
#include "ModularNeuralControl.h"
#include "BackboneJointControl.h"
#include "delayline.h"
#include "forwardmodel.h"
#include "motormapping.h"

//------------------Muscle parameters------------------------//
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


//------------------Muscle parameters------------------------//

//Save files
#include <iostream>
#include <fstream>
#include <string.h>
using namespace std;
//Save files

//3) Class for Neural locomotion control------

class NeuralLocomotionControlAdaptiveClimbing{

public:
    typedef double sensor;
	//---Start Define functions---//
	NeuralLocomotionControlAdaptiveClimbing(int aamosVersion);
	~NeuralLocomotionControlAdaptiveClimbing();




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






	double VirtualMusclesAcce1(double Ang, double DAng, double t, double Exforce, double MuscleAct, double DisVe, double K, double D, double Leng, double Theta1, double MusclActFac) //step function for FTi joints,
	//see RungeKutta41(…)
	      {


	        double ExVec,ExforceTor,PasiForceTor,ActiForceTor,NeedAngle;

	        ExVec = (Leng + Radius);//*cos(Ang) + DisVe;

	        ExforceTor = Exforce * sin(Ang) * ExVec;
	        PasiForceTor = (2 * ((D * DAng) + (K * Ang)) * Radius) * Radius;
	        ActiForceTor = MuscleAct * Radius *MusclActFac;

	        NeedAngle = (ExforceTor + ActiForceTor - PasiForceTor)/(Mass * Radius * Radius / 2.0);

	        return NeedAngle;

	      }

	      double VirtualMusclesAcceN(double Ang, double DAng, double t, double Exforce, double MuscleAct, double DisVe, double K, double D, double Leng, double Theta1, double MusclActFac) //step function for CTr // joints, see RungeKutta42(…)
	        {
	          double ExVec,ExforceTor,PasiForceTor,ActiForceTor,NeedAngle;

	          ExVec = (Leng + Radius) * cos(Ang) + DisVe;

	          ExforceTor = Exforce * ExVec;//Exforce * cos(Theta1) * ExVec;
	          PasiForceTor = (2 * ((D * DAng) + (K * Ang)) * Radius) * Radius;
	          ActiForceTor = MuscleAct * Radius * MusclActFac;

	          NeedAngle = (ExforceTor + ActiForceTor - PasiForceTor)/(Mass * Radius * Radius / 2.0);

	          return NeedAngle;

	        }

	      MusclePara RungeKutta41(double x, double v, double dt, double PreExforce, double Exforce, double PreMuscleAct, double MuscleAct, double DisVec, double K, double D, double Leng, double Theta1, double MusclActFac) // for FTi joints
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

	      MusclePara RungeKutta42(double x, double v, double dt, double PreExforce, double Exforce, double PreMuscleAct, double MuscleAct, double DisVec, double K, double D, double Leng, double Theta1, double MusclActFac) // for CTr joints
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




	//------------------Muscle definitions and functions------------------------//


	virtual void calculate(const std::vector<double> x,AmosIISensorNames sensorname,NeuralLocomotionControlAdaptiveClimbing * NeighbourNLC,int i,NeuralLocomotionControlAdaptiveClimbing * NLCAC[6],double aNeighbourCpg_activity_0,double aNeighbourCpg_activity_1,double* y);
	double sigmoid(double num)
	{
		return 1.0/(1.0+exp(-num));
	}

	double sgn(double num)
	{
		if(num>0)
		{
			return 1.0;
		}
		if(num<0)
		{
			return -1.0;
		}
		else
		{
			return 0;
		}
	}

	double pos(double num) {
		if(num > 0.0)
			return num;
		else
			return 0.0;
	}

	double neg(double num) {
		if(num < 0.0)
			return num;
		else
			return 0.0;
	}

	string MotorNames(int num) {
		switch(num){
		case 6:
			return "CR0_m";
			break;
		case 7:
			return "CR1_m";
			break;
		case 8:
			return "CR2_m";
			break;
		case 9:
			return "CL0_m";
			break;
		case 10:
			return "CL1_m";
			break;
		case 11:
			return "CL2_m";
			break;
		}
		return 0;
	}


	std::vector<double> step_nlc(const std::vector<double> inreflex,
	    const std::vector< vector<double> > in0, bool Footinhibition);
	// if several sensor values (like preprocessed or output of memory and learning are used, extend parameters)
	// std::vector<double> step_nlc(const std::vector<double> in0, const std::vector<double> in1, const std::vector<double> in2);

	//---End  Define functions---//



	//---Start Save files---//
	ofstream outFilenlc1;
	//---End Save files---//


	//---Start Define vector----//

	//---Reflex
	std::vector<double> rev_reflex;
	std::vector<double> rev_reflex_activity1;
	std::vector<double> rev_reflex_activity2;
	std::vector<double> rev_reflex_output1;
	std::vector<double> rev_reflex_output2;
	bool reversegear;

	//---IR Leg Refl Activation
	std::vector<double> irleg_activity;
	std::vector<double> irleg_output;
	std::vector<double> irleg_diff;
	double input_irl_w;
	double irl_irl_w;
	double irl_bias;
	std::vector<double> irl_restart;
	std::vector<double> irl_restart_diff;


	//TESTING
	std::vector<double> test_cpg_output;
	std::vector<double> test_pcpg_output;
	std::vector<double> test_psn_output;
	std::vector<double> test_vrn_output;
	std::vector<double> test_motor_activity;
	double test_sensorinput;
	double test_outputfinal;

	 std::vector<double> x_s;

	//---turning
	bool turning;
	int counter_turn;
// -----footbias
	// std::vector<double> footbias;

	//---Input neurons
	std::vector<double> input;

	//---MODULE 1 CPG
	std::vector<double> cpg_output;						//CPG neural outputs
	std::vector<double> cpg_Activity;
  std::vector<double> Neighbourcpg_output;
	double Control_input;								//Console
	//double GlobalNeighbourCpg_phase_phi;
  double NeighbourCpg_activity_0;
  double NeighbourCpg_activity_1;
  int CPGID;
  NeuralLocomotionControlAdaptiveClimbing * GlobalNeighbourNLCAC[6];

	//---MODULE 2 CPG POST-PROCESSING

	std::vector<double> pcpg_output;					//pCPG neural outputs



	//---MODULE 3 PSN

	std::vector<double> psn_output;						//PSN neural outputs

	//---MODULE 4 VRN

	std::vector<double> vrn_output;						//VRN neural outputs

	double vrn_outputM; ///to check VRN neural outputs
  double vrn_outputTanh;
	//---MODULE 5 Motor neurons

	std::vector<double> bj_activity;					//motor neural activities
	double bias_bjc;

	std::vector<double> tr_output; 				    	//motor neural outputs
	std::vector<double> tl_output;						//motor neural outputs
	std::vector<double> cr_output; 				    	//motor neural outputs
	std::vector<double> cl_output;						//motor neural outputs
	std::vector<double> fr_output; 				   	 	//motor neural outputs
	std::vector<double> fl_output;						//motor neural outputs
	std::vector<double> bj_output;						//motor neural outputs
	std::vector<double> tr_outputOld;
	std::vector<double> tl_outputOld;

	std::vector<double> postcr;							//postCL motor neural outputs
	std::vector<double> postcl;							//postCR motor neural outputs
	std::vector<double> postcrold;						//postCL motor neural outputs
	std::vector<double> postclold;						//postCR motor neural outputs
	std::vector<double> postcldelay;
	std::vector<double> postcrdelay;

	std::vector<double> m_pre;							//pre motor outputs (19 motors)
	std::vector<double> m_reflex;						//reflex motor outputs  (19 motors)
	std::vector<double> m;								//motor outputs as neural activation (19 motors)
	std::vector<double> m_deg;							//motor outputs in deg (19 motors)
	std::vector<double> delta_m_pre;       //delta of pre motor output (19 motors)


	//---Reflex motor neurons
	std::vector<double> fmodel_cmr_activity;			//coxa motor right neural activities
	std::vector<double> fmodel_cmr_output; 				//coxa motor right neural outputs
	std::vector<double> fmodel_error;			    //error coxa motor right and foot right
	std::vector<double> fmodel_lerror;         //error coxa motor right and foot right
	std::vector<double> fmodel_cmr_outputfinal; 		//coxa motor right neural outputs


	std::vector<double> fmodel_activity;			//coxa motor left neural activities
	std::vector<double> fmodel_cml_output; 				//coxa motor left neural outputs
	std::vector<double> fmodel_cml_error;			    //error coxa motor left and foot left
	std::vector<double> fmodel_cml_outputfinal; 		//coxa motor right neural outputs


	//---Backbone Joint Control Parameters
	double bj_w;
	double bj_rec_w;
	double bj_signal;
	double bjc_offset;



// -- LIFt the body
	double lifting_value;
	bool lift_body_up;
  std::vector<double> offset_ftil_downward;
  std::vector<double> offset_ftir_downward;
  std::vector<double> offset_ctl_downward;
  std::vector<double> offset_ctr_downward;

	std::vector<double> reflex_fs;


	//---Reflex IR leg sensors for duration
	std::vector<double> reflex_irs;

	//---Reflex speed sensors
	std::vector<double> reflex_speed;

	//Learning forward models to expected foot sensors
	std::vector<double> lr_fmodel_cr;					//learning rate
	std::vector<double> fmodel_cmr_w;					//forward model weights
	std::vector<double> fmodel_fmodel_w;      //forward model recurrent weights
	std::vector<double> fmodel_fmodel_cmr_w;			//forward model recurrent weights
	std::vector<double> fmodel_post_cmr_w;				//forward model postprocessing weights
	std::vector<double> fmodel_cmr_bias;				//forward model biases
	std::vector<double>  acc_cmr_error;					//forward model biases
	std::vector<double>  acc_cmr_error_old;				//forward model biases
	std::vector<double>  deri_acc_cmr_error;			//forward model biases
	std::vector<double>  acc_cmr_error_elev;			//error for elevator reflex
	std::vector<double>  error_cmr_elev;				//error for elevator reflex
	std::vector<double>  acc_cmr_error_posi_neg;    //forward model biases
	std::vector<double>  dervi_fmodel_cmr_output;
	double lowpass_error_gain;




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
	std::vector<double>  acc_cml_error_posi_neg;    //forward model biases
	std::vector<double>  dervi_fmodel_cml_output;

	std::vector<double> lowpass_cmr_error_activity;		//lowpass neuron activities
	std::vector<double> lowpass_cmr__error_output; 		//lowpass neuron outputs
	std::vector<double> lowpass_cmr_w;					//lowpass weights
	std::vector<double> lowpass_lowpass_cmr_w;			//lowpass recurrent weights
	std::vector<double> lowpass_cmr_bias;				//lowpass biases
	std::vector<double> low_pass_fmodel_cmr_error_old;
	std::vector<double> low_pass_fmodel_cmr_error;

	std::vector<double> lowpass_cml_error_activity;		//lowpass neuron activities
	std::vector<double> lowpass_cml__error_output; 		//lowpass neuron outputs
	std::vector<double> lowpass_cml_w;					//lowpass weights
	std::vector<double> lowpass_lowpass_cml_w;			//lowpass recurrent weights
	std::vector<double> lowpass_cml_bias;				//lowpass biases
	std::vector<double> low_pass_fmodel_cml_error_old;
	std::vector<double> low_pass_fmodel_cml_error;

	std::vector<double> fmodel_cmr_errorW;
	std::vector<double> fmodel_cml_errorW;
	std::vector<double> fmodel_cmr_output_old;
	std::vector<double> fmodel_cml_output_old;

	std::vector<double> acc_error; //forward model class output
	std::vector<double> fmodel_counter;
	std::vector<double> fmodel_b;
	std::vector<double> fmodel_w;
	std::vector<double> fmodel_output;
	std::vector<double> fmodel_outputfinal;
	std::vector<string> converge;

	//Neural Locomotion Control
	ModularNeuralControl* nlc;

	//Backbone Joint Control
	BackboneJointControl* bjc;


	std::vector< Forwardmodel* > fmodel;       //fmodel object vector
	std::vector< Mapping* > motormap;              //motor mapping object vector

	std::vector<double> ctr_oldvalue;

	//TEST

	double cpg_act_0;
	double cpg_act_1;
	double cpg_w_0;
	double cpg_w_1;
	double cpg_bias_0;
	double cpg_bias_1;
	double cpg_rec_0;
	double cpg_rec_1;

	bool switchon_backbonejoint;
	bool switchon_obstacle;
	bool switchon_reflexes;
	bool switchon_purefootsignal;
	bool switchon_irreflexes;
	bool switchon_allreflexactions;
	bool switchon_footinhibition;

	bool coxawithContactSensorSignalIsEnabled;
	int amosVersion;
	bool isMuscle;

	//---End Define vector----//

	const std::vector<double> xsensors;


private:
	NeuralLocomotionControlAdaptiveClimbing * NeighbourNLCGlobal;
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

	double bj_delta;

	double ext;

	int tau;
	int tau_l;
	int time;

	int option_wiring;
	std::vector<double> counter_cr;
	std::vector<double> counter_cl;

	bool prolong;
	int counter;
	int global_count;


	int allfoot_off_ground;


	int option_pcpg;
	int option_fmodel;
	int option_cpg;
	bool switchon_learnweights;
	bool softlanding;
	bool switchon_learningcycle;
	bool switchon_reactivebjc;

	enum{ //CPG option
		SO2cpg = 1,
		AdaptiveSO2cpg = 2,
		AdaptiveCpg = 3,// subhi
	};



};

#endif /* NEURALLOCOMOTIONCONTROLADAPTIVECLIMBING_H_ */
