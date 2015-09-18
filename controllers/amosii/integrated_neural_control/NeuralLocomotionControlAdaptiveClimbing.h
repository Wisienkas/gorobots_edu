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
#include "MuscleModel.h"
#include "ModularNeuralControl.h"
#include "BackboneJointControl.h"
#include "delayline.h"
#include "forwardmodel.h"
#include "motormapping.h"


//Save files
#include <iostream>
#include <fstream>
#include <string.h>
using namespace std;
//Save files

//3) Class for Neural locomotion control------

class NeuralLocomotionControlAdaptiveClimbing{

public:

	//---Start Define functions---//
	NeuralLocomotionControlAdaptiveClimbing();
	NeuralLocomotionControlAdaptiveClimbing(int aamosVersion,bool mMCPGs,bool mMuscleModel);
	~NeuralLocomotionControlAdaptiveClimbing();

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


	vector<double> step_nlc(const vector<double> inreflex, const vector< vector<double> > in0, bool Footinhibition=false);
	// if several sensor values (like preprocessed or output of memory and learning are used, extend parameters)
	// vector<double> step_nlc(const vector<double> in0, const vector<double> in1, const vector<double> in2);

	//---End  Define functions---//



	//---Start Save files---//
	ofstream outFilenlc1;
	//---End Save files---//


	//---Start Define vector----//


	MuscleModel* muscleModel;
	//---Reflex
	vector<double> rev_reflex;
	vector<double> rev_reflex_activity1;
	vector<double> rev_reflex_activity2;
	vector<double> rev_reflex_output1;
	vector<double> rev_reflex_output2;
	bool reversegear;

	//---IR Leg Refl Activation
	vector<double> irleg_activity;
	vector<double> irleg_output;
	vector<double> irleg_diff;
	double input_irl_w;
	double irl_irl_w;
	double irl_bias;
	vector<double> irl_restart;
	vector<double> irl_restart_diff;


	//TESTING
	vector<double> test_cpg_output;
	vector<double> test_pcpg_output;
	vector<double> test_psn_output;
	vector<double> test_vrn_output;
	vector<double> test_motor_activity;
	double test_sensorinput;
	double test_outputfinal;

	//---turning
	bool turning;
	int counter_turn;

	//---Input neurons
	vector<double> input;

	//---MODULE 1 CPG
	vector< vector<double> > cpg_output;						//CPG neural outputs

	double Control_input;								//Console

	//---MODULE 2 CPG POST-PROCESSING

	vector< vector<double> > pcpg_output;					//pCPG neural outputs



	//---MODULE 3 PSN

	vector< vector<double> > psn_output;						//PSN neural outputs

	//---MODULE 4 VRN

	vector< vector<double> > vrn_output;						//VRN neural outputs



	//---MODULE 5 Motor neurons

	vector<double> bj_activity;					//motor neural activities
	double bias_bjc;

	vector<double> tr_output; 				    	//motor neural outputs
	vector<double> tl_output;						//motor neural outputs
	vector<double> cr_output; 				    	//motor neural outputs
	vector<double> cl_output;						//motor neural outputs
	vector<double> fr_output; 				   	 	//motor neural outputs
	vector<double> fl_output;						//motor neural outputs
	vector<double> bj_output;						//motor neural outputs
	vector<double> tr_outputOld;
	vector<double> tl_outputOld;


	vector<double> postcr;							//postCL motor neural outputs
	vector<double> postcl;							//postCR motor neural outputs
	vector<double> postcrold;						//postCL motor neural outputs
	vector<double> postclold;						//postCR motor neural outputs


	vector<double> m_pre;							//pre motor outputs (19 motors)
	vector<double> m_reflex;						//reflex motor outputs  (19 motors)
	vector<double> m;								//motor outputs as neural activation (19 motors)
	vector<double> m_deg;							//motor outputs in deg (19 motors)
	vector<double> delta_m_pre;       //delta of pre motor output (19 motors)


	//---Reflex motor neurons
	vector<double> fmodel_cmr_activity;			//coxa motor right neural activities
	vector<double> fmodel_cmr_output; 				//coxa motor right neural outputs
	vector<double> fmodel_error;			    //error coxa motor right and foot right
	vector<double> fmodel_lerror;         //error coxa motor right and foot right
	vector<double> fmodel_cmr_outputfinal; 		//coxa motor right neural outputs


	vector<double> fmodel_activity;			//coxa motor left neural activities
	vector<double> fmodel_cml_output; 				//coxa motor left neural outputs
	vector<double> fmodel_cml_error;			    //error coxa motor left and foot left
	vector<double> fmodel_cml_outputfinal; 		//coxa motor right neural outputs


	//---Backbone Joint Control Parameters
	double bj_w;
	double bj_rec_w;
	double bj_signal;
	double bjc_offset;




	//---Reflex foot sensors
	vector<double> reflex_fs;


	//---Reflex IR leg sensors for duration
	vector<double> reflex_irs;

	//---Reflex speed sensors
	vector<double> reflex_speed;

	//Learning forward models to expected foot sensors
	vector<double> lr_fmodel_cr;					//learning rate
	vector<double> fmodel_cmr_w;					//forward model weights
	vector<double> fmodel_fmodel_w;      //forward model recurrent weights
	vector<double> fmodel_fmodel_cmr_w;			//forward model recurrent weights
	vector<double> fmodel_post_cmr_w;				//forward model postprocessing weights
	vector<double> fmodel_cmr_bias;				//forward model biases
	vector<double>  acc_cmr_error;					//forward model biases
	vector<double>  acc_cmr_error_old;				//forward model biases
	vector<double>  deri_acc_cmr_error;			//forward model biases
	vector<double>  acc_cmr_error_elev;			//error for elevator reflex
	vector<double>  error_cmr_elev;				//error for elevator reflex
	vector<double>  acc_cmr_error_posi_neg;    //forward model biases
	vector<double>  dervi_fmodel_cmr_output;
	double lowpass_error_gain;




	vector<double> lr_fmodel_cl;					//learning rate
	vector<double> fmodel_cml_w;					//forward model weights
	vector<double> fmodel_fmodel_cml_w;			//forward model recurrent weights
	vector<double> fmodel_post_cml_w;				//forward model postprocessing weights
	vector<double> fmodel_cml_bias;				//forward model biases
	vector<double>  acc_cml_error;					//forward model biases
	vector<double>  acc_cml_error_old;				//forward model biases
	vector<double>  deri_acc_cml_error;			//forward model biases
	vector<double>  acc_cml_error_elev;			//error for elevator reflex
	vector<double>  error_cml_elev;				//error for elevator reflex
	vector<double>  acc_cml_error_posi_neg;    //forward model biases
	vector<double>  dervi_fmodel_cml_output;

	vector<double> lowpass_cmr_error_activity;		//lowpass neuron activities
	vector<double> lowpass_cmr__error_output; 		//lowpass neuron outputs
	vector<double> lowpass_cmr_w;					//lowpass weights
	vector<double> lowpass_lowpass_cmr_w;			//lowpass recurrent weights
	vector<double> lowpass_cmr_bias;				//lowpass biases
	vector<double> low_pass_fmodel_cmr_error_old;
	vector<double> low_pass_fmodel_cmr_error;

	vector<double> lowpass_cml_error_activity;		//lowpass neuron activities
	vector<double> lowpass_cml__error_output; 		//lowpass neuron outputs
	vector<double> lowpass_cml_w;					//lowpass weights
	vector<double> lowpass_lowpass_cml_w;			//lowpass recurrent weights
	vector<double> lowpass_cml_bias;				//lowpass biases
	vector<double> low_pass_fmodel_cml_error_old;
	vector<double> low_pass_fmodel_cml_error;

	vector<double> fmodel_cmr_errorW;
	vector<double> fmodel_cml_errorW;
	vector<double> fmodel_cmr_output_old;
	vector<double> fmodel_cml_output_old;

	vector<double> acc_error; //forward model class output
	vector<double> fmodel_counter;
	vector<double> fmodel_b;
	vector<double> fmodel_w;
	vector<double> fmodel_output;
	vector<double> fmodel_outputfinal;
	vector<string> converge;



	//Neural Locomotion Control
	vector< ModularNeuralControl* > nlc;

	//Backbone Joint Control
	BackboneJointControl* bjc;

	//Delayline constructor
	Delayline* tr_delayline;
	Delayline* tl_delayline;
	Delayline* ctr_delayline;
	Delayline* fti_delayline;
	Delayline* ftim_delayline;
	vector< Forwardmodel* > fmodel;       //fmodel object vector
	vector< Mapping* > motormap;              //motor mapping object vector

	vector<double> ctr_oldvalue;

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

	int amosVersion;
	unsigned int num_cpgs;
	bool MCPGs;
	bool muscleModelIsEnabled;
	int CPGID;

	void init(int aamosVersion,bool mMCPGs,bool mMuscleModel);
	double lift_value;
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
	vector<double> triH1; //triangle output vector
	vector<double> triH2;

	double bj_delta;

	double ext;

	int tau;
	int tau_l;
	int time;

	int option_wiring;
	vector<double> counter_cr;
	vector<double> counter_cl;

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
	};



};

#endif /* NEURALLOCOMOTIONCONTROLADAPTIVECLIMBING_H_ */
