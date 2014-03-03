/***************************************************************************
 *   Copyright (C) 2008 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   ANY COMMERCIAL USE FORBIDDEN!                                         *
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
 *   Meta controller allowing to choose between "manual", random and       *
 *   homeokinetic                                                          *
 *   control for each of the 19 degrees of freedom.                        *
 *                                                                         *
 *   $Log: amosIImetacontroller.h,v $                                *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/
#ifndef NIMM_OG_H
#define NIMM_OG_H

#include <selforg/abstractcontroller.h>

#include <assert.h>
#include <cmath>//test

#include <selforg/matrix.h>
#include <selforg/noisegenerator.h>

#include <vector>

//#include "openinvertnchannelcontroller.h"
//#include "qlearning.h"

//Save files
#include <iostream>
#include <fstream>
#include <string.h>
using namespace std;
//Save files


//----AC network parameters------------//

#define XDIM 3//4 // number of inputs ** Set for different system
#define UDIM 2//2 // number of output ** Set for different system
#define WDIM UDIM*XDIM//8 // number of actor weight ** Set for different system

#define N_IN0_RBF 3 //4 number of RBF on input 0 ** Set for different system
#define N_IN1_RBF 3 //4 number of RBF on input 1 ** Set for different system
#define N_IN2_RBF 3 //4 number of RBF on input 1 ** Set for different system
#define N_IN3_RBF 3 //4 number of RBF on input 1 ** Set for different system


#define MAX_IN0 M_PI//2.0//M_PI/2  //2 M_PI/2 (1.578) // max sensory input0 range ** Set for different system
#define MIN_IN0 -M_PI//-2.0//-M_PI/2 //-2 -M_PI/2 (1.578) // min sensory input0 range ** Set for different system

#define MAX_IN1 M_PI//M_PI//2.0//1.0 // max sensory input1 range ** Set for different system
#define MIN_IN1 -M_PI//-2.0//0.0 // min sensory input1 range ** Set for different system

//---NOT USED in this setup
#define MAX_IN2 M_PI//3.0  // max sensory input1 range ** Set for different system
#define MIN_IN2 -M_PI // min sensory input1 range ** Set for different system

#define MAX_IN3 M_PI//3.0  // max sensory input1 range ** Set for different system
#define MIN_IN3 -M_PI // min sensory input1 range ** Set for different system

#define _ias0 0 // name of input
#define _ias1 1 // name of input
#define _ias2 2 // name of input
#define _ias3 3 // name of input

#define print 1 // set to 1 = print for debug, 0 = no print

//Learning parameters

#define MAX_ITER  0.25*(60.0*200.000)// 0.25 = 30 s, 0.5 = 1 min, 1.0 = 2 mins, 1.5 = 3 mins
#define MAX_TRIAL 100000


//----AC network parameters------------//




#define NUMBER_OF_BALLS 4

struct NimmOgConf{
} ;



class Sample {
public:
	std::vector<double> inputs;
	std::vector<double> tds;
	double perror;

	~Sample() {
		inputs.clear();
		tds.clear();
	}
};


/**
 * class for robot controller 
 * using several controllers, see comments for the configuration above
 */
class NimmOG : public AbstractController {

public:

	vector<Sample> samples;
	double w_ico;
	double w_ac;
	double rate;
	double ss_rate;

	double lowpass_dis_x;
	double lowpass_dis_y;
	double lowpass_dis_x_old;
	double lowpass_dis_y_old;

	bool learn_combined_weights;

	double u_ico_lowpass;
	double ut_lowpass;

	double u_ico_old;
	double ut_old;

	double reset_data;
	int  nrepeat;
	unsigned int stepnumber_t;

	double reduce_with_time;
	bool hit_the_goal;
	double reduce_with_time_counter;
	//Save files
	ofstream outFileacico;
	ofstream outFiletd;
	ofstream outFilevtcurve;
	ofstream outFileicolearning;
	ofstream outFileiter;
	//added:
	ofstream outFileCalibrate;
//	ofstream outFilevtcurveGR;
//	ofstream outFilevtcurveG;
//	ofstream outFilevtcurveB;
//	ofstream outFilevtcurveY;
	//Save files
	double actual_out;
	double actor_out;
	double exploration_out;
	//////////////////////update actor
	double *input_old;
	double exploration_old;
	double verror;

	double distances [NUMBER_OF_BALLS];
	double low_pass_angles[NUMBER_OF_BALLS];
	double low_pass_angles_old[NUMBER_OF_BALLS];



	NimmOG(const NimmOgConf& conf = getDefaultConf());
	virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

	virtual ~NimmOG();

	void reset_params();

	/// returns the number of sensors the controller was initialised with or 0 if not initialised
	virtual int getSensorNumber() const { return number_sensors; }
	/// returns the mumber of motors the controller was initialised with or 0 if not initialised
	virtual int getMotorNumber() const  { return number_motors; }

	//virtual Position getPosition();

	/// performs one step (includes learning).
	/// Calulates motor commands from sensor inputs.
	virtual void step(const sensor* , int number_sensors, motor* , int number_motors);

	/// performs one step without learning. Calulates motor commands from sensor inputs.
	//
	virtual void stepNoLearning(const sensor* , int number_sensors,
			motor* , int number_motors);

	void get_v_action_pairs(double* in, double* out);
	void save_inputs();
	/************** CONFIGURABLE ******************************** /
  virtual paramval getParam(const paramkey& key) const;
  virtual bool setParam(const paramkey& key, paramval val);
  virtual paramlist getParamList() const;
	 */

	/**** STOREABLE ****/
	/** stores the controller values to a given file. */
	virtual bool store(FILE* f) const { return true; };
	/** loads the controller values from a given file. */
	virtual bool restore(FILE* f){ return true;};


	void save_output();

	static NimmOgConf getDefaultConf(){
		NimmOgConf c;
		//TODO: Do something here!
		return c;
	}

	void copy_input(double* inputs, double* inputs_old);

	void setProgrammedSteeringControl(bool b){
		preprogrammed_steering_control_active = b;
	}

	void setLearnedSteeringControl(bool b){
		learned_steering_control_active = b;
	}

	void printQTable(){
		//qlearner->printQTable();
	}

	void setCurrentCycle(int currentCycle){
	   nrepeat = currentCycle;
	}

	void get_ir_signals(const sensor* x_);
	void print_to_cout(int steps_period);

	void setExplorationActive(bool b){
	//	qlearner->setExplorationActive(b);
	}
	void setMC(double left, double right){
		mc[0]=left;
		mc[1]=right;
		mc[2]=left;
		mc[3]=right;
	}

	void resetlearing(int reset)
	{
		resetLearning_RL = reset;
	}

	void position(int position_x_sim)
	{
		position_x_robot = position_x_sim;
	}

	//-----ICO Learning------------------//

	double sigmoid(double num);
	double tanh(double num);
	double xt_ico[XDIM];
	double xt_ico2[XDIM];
	double xt_ico3[XDIM];
	double xt_ico4[XDIM];

	double xt_ir;
	double xt__reflex_ir;

	double reward0;
	double reward1;


	double ico_input;
	double distance;
	double alpha_tmp;
	double alpha;
	double deri_alpha;
	double alpha_old;

	double distance2;
	double alpha_tmp2;
	double alpha2;
	double deri_alpha2;
	double alpha_old2;

	double distance3;
	double alpha_tmp3;
	double alpha3;
	double deri_alpha3;
	double alpha_old3;

	double distance4;
	double alpha_tmp4;
	double alpha4;
	double deri_alpha4;
	double alpha_old4;

	unsigned int current_state;
	unsigned int current_stateACICO;

	double output_actor;
	std::vector<double> input_angle_s; //input angle sensors
	std::vector<double> ias_w; 		   //weights of input angle sensors
	std::vector<double> etrace_io; 	   //etrace of input and output of RL
	std::vector<double> etrace_io_old; //old etrace of input and output of RL
	double input_distance_s;
	double input_distance_s2;
	double input_distance_s3;
	double input_distance_s4;

	double pos_x; // Global positions of the robot (x coordinate)
	double pos_y;
	double pos_z;

	double ir_l; // +
	double ir_r; //-
	double input_ir_s;

	double rate_ico;
	double 	deri_reflex_irl;
	double 	deri_reflex_irr;
	double xt_reflex_irl;
	double xt_reflex_irr;
	double xt_reflex_irl_old;
	double xt_reflex_irr_old;
	double k_ico[4];            //TODO ask Koh what all the xt and k_ arrays do

  double xt_reflex_angle;     //These Angles are set to non-zero only if the Agent is close to the respective target.
  double xt_reflex_angle2;    //all in all they only influence k_ico[]
  double xt_reflex_angle3;
  double xt_reflex_angle4;
	double deri_xt_reflex_angle;
	double xt_reflex_angle_old;
	double deri_xt_reflex_angle2;
	double xt_reflex_angle_old2;
	double deri_xt_reflex_angle3;
	double xt_reflex_angle_old3;
	double deri_xt_reflex_angle4;
	double xt_reflex_angle_old4;

	double u_ico_in[4];
	double u_ico[2];
	double y_ico[4]; //motor outputs
	int failure_flag_ico;    //initialized only once across several calls
	int reduce_noise;

	double xt_ico_lowpassold;
	double xt_ico_lowpass;
	double xt_ico_lowpass2old;
	double xt_ico_lowpass2;
	double xt_ico_lowpass3old;
	double xt_ico_lowpass3;
	double xt_ico_lowpass4old;
	double xt_ico_lowpass4;
	double range_reflex;

	double exp_output_decay_ICO;
	int reduce_noise_ICO;

	//-----ICO Learning------------------//

	void calculateAnglePositionFromSensors(const sensor* x_);
	void calculateDistanceToGoals(const sensor* x_);
  void inputSensorSignals(const sensor* x_);
  void calculateAngleReflexSignals(const sensor* x_);
	//----AC network parameters------------//

    //--------------ACTOR COMPINENTS-function()----------------------//
	double reset_position;
	//--Actor policy
	void output_policy(double *x, double *u);
	//--Exploration
	//--Uniform distribution noise
	double gauss();
	//--Reward
	double reward_function(double *x /*state*/, double *u /*action*/);//reward_function(double alpha, double *u);
	//--Check state fail?
	int check_limit(double *x);//(double alpha);
	//int check_limit(double *x, double irr_back, double irl_back);

	//--Update actor output trace
	void update_policy(double td_error, double rate);
	void update_policy_actor(double td_error, double rate,double* input);

	//--------------CRITIC COMPINENTS-function()--------------------//
	//--Init RBF
	void init_funcapprox();
	//--Value function
	double value_function(double *x);
	//--Update value function trace
	void update_valuefunction_trace(double *x);
	//--Update value function Learning mechanism of Critic!!
	void update_valuefunction(double *x, double td, double rate);
	void submit_sample(double* in, double td_v, double td_actor, double perror);
	//--------------Exploration gradient------------------//
	double get_exploration(double *net_out);
	void update_rbf_network(double* input_old, double td_error, double td_actor, double rate_valuefunction, double p_error, double rate_policy);
	void conduct_updates();
	//help procedures

	void print_RBF_network_to_file(int epoch);


	/////////////////////////params
	int termination_counter;
	int last_epoch;
	bool terminate_now;

	int absorbed_reward;

	int nbasis;
	int basis[XDIM /*=4 inputs, Set for different system */];
	double wbasis[XDIM /*=4 inputs, Set for different system */];
	int xdim; // input dimension
	int udim; // output dimension
	double xmin[XDIM /*=4 inputs, Set for different system */];
	double xmax[XDIM /*=4 inputs, Set for different system */];
	double basis_width[XDIM /*=4 inputs, Set for different system */];


	int nbasis_value;
	double xmin_value[XDIM /*=4 inputs, Set for different system */];
	double xmax_value[XDIM /*=4 inputs, Set for different system */];
	int basis_value[XDIM /*=4 inputs, Set for different system */];
	double wbasis_value[XDIM /*=4 inputs, Set for different system */];
	double basis_width_value[XDIM /*=4 inputs, Set for different system */];

	double acum_reward;  //initialized only once across several calls
	double balance_time; //initialized only once across several calls
	int failure_flag;    //initialized only once across several calls
	double threshold_r;

	double balance_time_log[MAX_TRIAL];
	double sig_out_log[MAX_TRIAL];
	double sig_out[UDIM];
	double sig[UDIM]; 	   //exploration
	double sig_elig[UDIM]; //exploration gradient
	double output_grad[UDIM];

	int ntrial;
	double param_log[MAX_TRIAL][XDIM];

	//--output_policy()--//
	double xt[XDIM];
	double ut[UDIM];
	double old_ut;
	double k[WDIM]; //Weights of actor = XDIM*UDIM
	double si;
	double exp_output[UDIM];

	double output_elig[WDIM];
	int sign_control;


	//--Learning parameters--//
    //--CRITIC
	double  td_limit;
	double  gam; // discount factor of value function
	double  lambda_v; // e trace of v value
	double  rate_valuefunction;// learning rate of critic
	double  error_thresh_critic; // error threshold for changing weights of critic
	double  unit_activation_thresh_critic; // threshold of critic unit for changing weights
	double  error_thresh_actor; // error threshold for changing weights of critic
	double  unit_activation_thresh_actor; // threshold of critic unit for changing weights
	double  Vt;
	double  td_error_old;
	double  deri_td_error;
	double td_error;
    double rt;

	//--ACTOR
	double  lambda_p;  // e trace of actor
	double  rate_policy; // learning rate of actor

	bool use_Learned_weightsv2;
	bool large_input_rangev2;

	//No learning noise
	bool no_learning_noise;

	//change from POMDP to fullobservable FLAG value
	int scenario_flag;

	//Toggle between RBF critic and ESN critic Flag value
	bool ESN_critic;

	//sensor1 -> angle sensor
	double theta_limit;
	double max_dis;
	double max_dis2;
	double max_dis3;
	double max_dis4;


	//additional
	double th_exp_td_vt;
	double time_to_test_learned_policy;

	double y_ac[4]; //motor outputs


	double ut_ac_ico[UDIM];

	//----AC network parameters------------//


	//----Exploration noise---------------//

	double exploration[UDIM];
	double exploration_lowpass[UDIM];
	double exploration_lowpass_old[UDIM];
	double exploration_g[UDIM];
	double exploration_lowpass_g[UDIM];
	double exploration_lowpass_old_g[UDIM];
	double exp_output_decay;

	//-----Sensory preprocessing----------//
	double irr_lowpass;
	double irl_lowpass;
	double irr_lowpass_old;
	double irl_lowpass_old;

	double irr_back;
	double irl_back;



protected:
	NimmOgConf conf;

	unsigned short number_sensors;
	unsigned short number_motors;
	unsigned short number_motors_homeokinetic;
	unsigned short number_sensors_homeokinetic;

	int t;
	bool preprogrammed_steering_control_active;
	bool learned_steering_control_active;

	unsigned int old_state, old_action;

//	QLearning* qlearner;
	double mc[4];

	int resetLearning_RL;
	int position_x_robot;
	int checkreset;

	//----Protecting Reset parameters------//
	bool initialized;//-------------------------------(FRANK)
	//----Protecting Reset parameters------//

};

#endif
