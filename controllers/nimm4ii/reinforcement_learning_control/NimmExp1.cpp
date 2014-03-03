/*************************************************************************
 * Reinforcement learning controller for nimm robot in
 * multiple goal scenario
 * Created/modified by Bassel Zeidan Dec 10, 2013
 **************************************************************************/
//General information
/*************************************************************************
 * Note: 1) one RBF network (with two outputs) is used for the actor and the critic to speed up the execution.
 * 		 However, the learning rule is different for these two outputs.
 *		 2) In each learning iteration, the learning samples (TD error, inputs, outputs) for each time step
 *		 	are saved in a list called (samples) until the termination of the iteration and then the updates
 *		 	are done over the RBF network (learning in a batch mode)
 *		 	submit_sample() ---> function to save a learning sample (called at each time step)
 *		 	update_critic_actor() ---> perform updates over the RBF network (called after the learning iteration is terminated)
 *
 * Main variables:
 * 		RBF network --> ngnet
 *		current reward variable ---> rt
 *		current V value ----> Vt
 *		actor learning rate ----> rate_policy
 *		Critic learning rate ----> rate_valuefunction
 *		discount factor -----> gam
 *		TD error -----> td_error
 *		current exploration -----> exploration_out
 *		the actor's output -----> actor_out
 *		the actual output of the system ------> actual_out
 *		sample list ----> samples
 *
 * Reward function:
 * 		1 when the robot reaches the right subgoal.
 * 		-1  when the robot hits an obstacle
 *
 * Parameters:
 * 		The actor's learning rate = 0.01
 * 		The critic's learning rate = 0.05
 * 		The discount factor = 0.9999
 * 		exploration scale factor = 0.2
 *
 *
 * Actor:
 * 		Number of inputs: 4
 * 		inputs: the relative angle from three subgoals (blue, Green, red)
 * 		the fourth input is the SD input (Subgoal definer)
 * 		Number of hidden neurons: 625
 * 		Number of outputs: one control output. (the second output of the RBF network)
 *
 * Critic:
 * 		Number of inputs: 4
 * 		inputs: the relative angle from three subgoals (blue, Green, red)
 * 		the fourth input is the SD input (Subgoal definer)
 * 		Number of hidden neurons: 625
 * 		Number of outputs: one output (the current V value) which is the first output of the RBF network
 *
**************************************************************************/
//main control section
/*************************************************************************************
 * * USE_SAVED_RBF: defines (1) using the saved RBF networks
 *   or (0) starting a new learning process.
 * * WRITE_RBF_TO_FILE: (1) writes RBF networks to a file after learning
 * * WRITE_CRITIC_ACTOR_WEIGHTS_TO_TEXT_FILE: (1) writes the weights of the actor
 *   and the critic at the end of each iteration
 * * TERMINATE_AFTER_LEARNING: (1) terminates the program after learning
 *************************************************************************************/
#define WRITE_RBF_TO_FILE 0
#define USE_SAVED_RBF 1
#define WRITE_CRITIC_ACTOR_WEIGHTS_TO_TEXT_FILE 0
#define TERMINATE_AFTER_LEARNING 0


#include "NimmExp1.h"
#include <selforg/controller_misc.h>
#include "rbf-framework/ngnet.h"

NGNet* ngnet;


#define rbf_num_units 625
#define rbf_num_IN 4
#define rbf_num_out 2


Cell VALUE(rbf_num_units, rbf_num_IN, rbf_num_out);

#ifndef clip
#define	clip(x/*input*/,l /*lower limit*/,u /*upper limit*/)( ((x)<(l)) ? (l) : ((x)>(u)) ? (u) : (x))
#endif

#ifndef min
#define min(x/*e.g., 1*/, y) ((x <= y) ? x : y)
#endif

#ifndef max
#define max(x/*e.g., 0*/, y) ((x >= y) ? x : y)
#endif

using namespace matrix;
using namespace std;
int current_epoch;


NimmExp1::NimmExp1(const NimmExp1Conf& _conf)
: AbstractController("NimmExp1", "$Id: "), conf(_conf)
{
	td_error=0.0;
	current_epoch= 0;
	ngnet = new NGNet(rbf_num_IN, rbf_num_out);
	preprogrammed_steering_control_active =false;
	t=0;
	mc[0]=0;
	mc[1]=0;
	mc[2]=0;
	mc[3]=0;
	resetLearning_RL = 2;
	output_actor = 0.0;
	input_angle_s.resize(XDIM);
	ias_w.resize(2);
	etrace_io.resize(2);
	etrace_io_old.resize(2);


	//-----Plot values----------------------//

	addInspectableValue("VT",&real_v,"Vt");
	addInspectableValue("rt",&rt,"rt");
	addInspectableValue("TD_Error",&td_error,"TD_Error");
	addInspectableValue("perror",&p_error,"perror");
	addInspectableValue("exploration_out",&exploration_out,"exploration_out");
	addInspectableValue("actual_out",&actual_out,"actual_out");
	addInspectableValue("actor_out",&actor_out,"actor_out");
	addInspectableValue("xt[_ias0]",&xt[_ias0],"xt[_ias0]");
	addInspectableValue("xt[_ias1]",&xt[_ias1],"xt[_ias1]");
	addInspectableValue("xt[_ias2]",&xt[_ias2],"xt[_ias2]");
	addInspectableValue("xt[_ias3]",&xt[3],"xt[3]");
	initialized = false; //-----------------------------------------------(FRANK)
	use_Learned_weightsv2 = false;//--------------true-----------------------------------------------TEST
	large_input_rangev2 = false;
	no_learning_noise = true;
};
NimmExp1::~NimmExp1()
{
}
//initialization function
void NimmExp1::init(int sensornumber, int motornumber, RandGen* randGen)
{
	stepnumber_t = 0;
	++current_epoch;
	if(!initialized) //-----------------------------------------------(FRANK)
	{
		terminate_immediatly = false;
		w_ico = 0.5;
		w_ac= 0.5;
		rate = 0.0005;
		ss_rate = 0.0005;
		terminate_after_this_step = false;
		learn_combined_weights = true; //true ;
		reduce_noise = 0;
		reduce_noise_ICO = 0;
		number_sensors = sensornumber;
		number_motors = motornumber;
		old_state=0;
		old_action=0;
		deri_alpha = 0.0;
		alpha_old = 0.0;
		alpha_old2 = 0.0;
		alpha_old3 = 0.0;
		alpha_old4 = 0.0;
		k_ico[0] = 0.0;
		k_ico[1] = 0.0;
		k_ico[2] = 0.0;
		k_ico[3] = 0.0;
		reduce_noise = 0;
		xt_reflex_angle = 0.0;
		deri_xt_reflex_angle = 0.0;
		xt_reflex_angle_old = 0.0;

		xt_reflex_angle2 = 0.0;
		deri_xt_reflex_angle2 = 0.0;
		xt_reflex_angle_old2 = 0.0;

		xt_reflex_angle3 = 0.0;
		deri_xt_reflex_angle3 = 0.0;
		xt_reflex_angle_old3 = 0.0;

		xt_reflex_angle4 = 0.0;
		deri_xt_reflex_angle4 = 0.0;
		xt_reflex_angle_old4 = 0.0;


		//----AC network parameters------------//
		//Initial only one time when start system
		nbasis =0;
		nbasis_actor =0;
		xdim = XDIM /*=2 inputs, Set for different system */;
		udim = UDIM;/*output of critic always = 1, Set for different system */;
		init_funcapprox();

		if (USE_SAVED_RBF) {
			nbasis = rbf_num_units;
			VALUE.read_file(rbf_num_units, rbf_num_IN, rbf_num_out);
			//VALUE.print_ws(rbf_num_units, rbf_num_IN, rbf_num_out);
		}

		acum_reward = 0.0;  //initialized only once across several calls
		balance_time = 0.0; //initialized only once across several calls
		failure_flag = 0;   //initialized only once across several calls
		reset_position = 0; //NOT use
		ntrial = 0;
		sig_out[0]= 0.0;
		nrepeat = 0;

		// initial weight of actors
		for(int i = 0; i< WDIM/*8*/; i++)
		{
			k[i] = 0.0;
		}

		//Exploration gradient output
		for(int i =0; i< UDIM; i++)
		{
			exploration[i]=0.0;
			exploration_lowpass[i]=0.0;
			exploration_lowpass_old[i]=0.0;
			exploration_g[i]=0.0;
			exploration_lowpass_g[i]=0.0;
			exploration_lowpass_old_g[i]=0.0;
		}

		for(int i =0; i< WDIM; i++)
		{
			output_elig[i]=0.0;
		}

		si = 1.0;//10.0;+++
		rate_policy = 0.01; //0.001;//0.01;//0.01;// learning rate of actor
		rate_valuefunction = 0.05;// learning rate_critic of weights from hidden RBF neurons to its output neuron (critic)

		//--CRITIC
		td_limit = 1.0; // limit of clipping TDerror (critic)
		gam = 0.9999;//0.99999999999;//0.98;//0.98; // discount factor of Vt (critic)
		error_thresh_critic = 0.5; // threshold for critic weight changes
		unit_activation_thresh_critic = 0.5;//0.3;//0.6; // threshold for critic weight changes

		error_thresh_actor = 0.5;
		unit_activation_thresh_actor = 0.5;

		lambda_v = 0.0;//0.9;//0.7;// E trace of Vt value: 0.0<= lambda_v < 1.0, if lambda_v ~> 1.0 = high low pass, if lambda_v ~> 0.0 = no low pass (critic)
		Vt = 0.0;
		td_error_old = 0.0;
		deri_td_error = 0.0;


		//--ACTOR
		lambda_p = 0.0;//0.9;  // E trace of actor output for weight adaptation
		sign_control = 1;

		//--REWARD
		threshold_r = 15;//5;//20; //Set threshold !! for learning if beyond this threshold the robot gets punishment!
		theta_limit = M_PI/2; // 15 deg

		//Threshold of TDerror, Vt, Exploration
		th_exp_td_vt = 0.00001;
		time_to_test_learned_policy = 50;//500;// steps

		//----AC network parameters------------//

		//---ICO learning----------------------//
		rate_ico = 0.005;   // 0.005;// ICO         0.001; // ICO_AC
		range_reflex = 0.2;

		checkreset = 0;
		initialized = true; //-----------------------------------------------(FRANK)
		for (int i=0; i<NUMBER_OF_BALLS; i++) {
			taken_rewards[i] = 0;
			reached_goals[i] = 0;
		}

		printf("Initial policy K0 = %f, K1 = %f \n\n", k[0], k[1]);

		//--preprocessing init-----------------//
		irl_lowpass_old = 0.0;
		irr_lowpass_old = 0.0;
		irl_lowpass = 0.0;
		irr_lowpass = 0.0;
		ir_l = 0.0;
		ir_r = 0.0;

		xt_reflex_irl = 0.0;
		xt_reflex_irr = 0.0;
		xt_reflex_irl_old = 0.0;
		xt_reflex_irr_old = 0.0;
		deri_reflex_irl = 0.0;
		deri_reflex_irr = 0.0;

		xt_ico_lowpassold = 0.0;
		xt_ico_lowpass = 0.0;
		xt_ico_lowpass2old = 0.0;
		xt_ico_lowpass2 = 0.0;
		xt_ico_lowpass3old = 0.0;
		xt_ico_lowpass3 = 0.0;
		xt_ico_lowpass4old = 0.0;
		xt_ico_lowpass4 = 0.0;

		exp_output_decay_ICO = 1.0;


		for(int i =0; i< 4; i++)
		{
			y_ac[i] = 0.0;
			y_ico[i] = 0.0;
		}
		hit_the_goal= false;
		reduce_with_time_counter = 0;
		exploration_old = 0;
		input_old = new double[XDIM];
		for (int i=0; i< XDIM; i++)
			input_old[i] = 0;
	}

}
//calculate relative angles
void NimmExp1::calculateAnglePositionFromSensors(const sensor* x_)
{
	int i=0;
	for (int counter=0; counter<NUMBER_OF_BALLS; ++counter)
	{
		//std::cout<<"iam here="<<x_[12 + i]<<", "<<x_[13 + i]<<std::endl;
		double alpha_value = 0;
		if (sign(x_[12 + i])>0)
			alpha_value = atan(x_[13 + i]/x_[12 + i]) * 180 / M_PI; // angle in degrees
		else {
			double alpha_value_tmp = -1*atan (x_[13 + i]/x_[12 + i]) * 180 / M_PI; // angle in degrees
			if (alpha_value_tmp<=0)
				alpha_value = (-90 + (-90-alpha_value_tmp));
			else alpha_value = ( 90 + ( 90-alpha_value_tmp));
		}
		input_angle_s.at(counter) = alpha_value/180.*M_PI;
		i += 3;
	}
}
//calculate distances
void NimmExp1::calculateDistanceToGoals(const sensor* x_)
{
	double distance_scale = 0.001;//1/100;

	max_dis = position_x_robot*position_x_robot; //normalize
	distance = (sqrt(pow(x_[13/*11 y*/],2)+pow(x_[12/*10 x*/],2)));
	//input_distance_s = distance*distance_scale;//85;//max_dis;
	distances[0] = distance*distance_scale;//85;//max_dis;
	//input_distance_s -= 2;

	max_dis2 = position_x_robot*position_x_robot; //normalize
	distance2 = (sqrt(pow(x_[16/*y*/],2)+pow(x_[15/*x*/],2)));
	//input_distance_s3 = distance2*distance_scale;//85;//max_dis;
	distances[1] = distance2*distance_scale;//85;//max_dis;
	//input_distance_s2 -= 2;

	max_dis3 = position_x_robot*position_x_robot; //normalize
	distance3 = (sqrt(pow(x_[19/*y*/],2)+pow(x_[18/*x*/],2)));
	distances[2] = distance3*distance_scale;//85;//max_dis;
	//input_distance_s2 = distance3*distance_scale;//85;//max_dis;

}
//low pass filters for input signals
void NimmExp1::inputSensorSignals(const sensor* x_)
{
	xt_ir = input_ir_s;

	xt_ico_lowpassold = xt_ico_lowpass;
	xt_ico_lowpass2old = xt_ico_lowpass2;
	xt_ico_lowpass3old = xt_ico_lowpass3;

	double gain_r = 0.95;
	xt_ico_lowpass = ((1-gain_r)*input_angle_s.at(0)) + (xt_ico_lowpassold*gain_r);//red
	xt_ico_lowpass2 = ((1-gain_r)*input_angle_s.at(1)) + (xt_ico_lowpass2old*gain_r); //*** USED //Goal Green 2
	xt_ico_lowpass3 = ((1-gain_r)*input_angle_s.at(2)) + (xt_ico_lowpass3old*gain_r); //*** USED //Goal Blue 3

	///////////////////////////////////////////////////////////
	/////////////distances

	double gain_ir = 0.99;//0.99;

	lowpass_dis_x_old = lowpass_dis_x;
	lowpass_dis_y_old = lowpass_dis_y;

	lowpass_dis_x = (1-gain_ir)*((x_[12])/120.0)+lowpass_dis_x_old*gain_ir;
	lowpass_dis_y = (1-gain_ir)*((x_[13])/120.0)+lowpass_dis_y_old*gain_ir;

}
//low pass filter for IR signals
void NimmExp1::get_ir_signals(const sensor* x_)
{
	//---4) Calculating IR sensors for obstacle detection
	ir_l = (x_[5]+x_[7]+x_[6]);//(x_[5]+x_[6]+x_[7]); // +
	ir_r = (x_[4]+x_[11]+x_[10]);//(x_[4]+x_[8]+x_[9]); // -

	input_ir_s = ir_l-ir_r;

	//preprocessing using a low pass filter
	double gain_ir = 0.99;

	irl_lowpass_old = irl_lowpass;
	irr_lowpass_old = irr_lowpass;

	irl_lowpass = (1-gain_ir)*ir_l+irl_lowpass_old*gain_ir;
	irr_lowpass = (1-gain_ir)*ir_r+irr_lowpass_old*gain_ir;
}
//printing function (NOT USED)
void NimmExp1::print_to_cout(int steps_period)
{
	if (stepnumber_t % steps_period == 0)
	{
		std::cout<<"epoch  -------------->"<<current_epoch<<std::endl;
		std::cout<<"Critic:\nerror threshold:"
				 <<error_thresh_critic<<std::endl
				 <<"Unit activation threshold:"
				 <<unit_activation_thresh_critic<<std::endl
				 <<"nbasis -------> "<<nbasis<<std::endl
				 <<"learning rate -------> "<<rate_valuefunction
				 <<"\n";
		std::cout<<"------------------------------------------------------\n";
		std::cout<<"Actor:\nerror threshold:"
				 <<error_thresh_actor<<std::endl
				 <<"Unit activation threshold:"
				 <<unit_activation_thresh_actor<<std::endl
				 <<"nbasis -------> "<<nbasis_actor<<std::endl
				 <<"learning rate -------> "<<rate_policy
				 <<"\n";
		std::cout<<"-------------------------------------------------------\n";
		std::cout<<"Statistics:"<<std::endl;
		std::cout<<"x[0]:"<<xt[0]<<std::endl;
		std::cout<<"x[1]:"<<xt[1]<<std::endl;
		std::cout<<"x[2]:"<<xt[2]<<std::endl;
		std::cout<<"x[3]:"<<xt[3]<<std::endl;
		std::cout<<"Current Reward: "<<rt<<std::endl;
		std::cout<<"Current V value: "<<Vt<<std::endl;
		std::cout<<"TD Error: "<<td_error<<std::endl;
		std::cout<<"exploration:"<<exp_output[0]<<std::endl;
		std::cout<<"reduce_with_time:"<<reduce_with_time<<std::endl;
		std::cout<<"reduce_with_time_counter:"<<reduce_with_time_counter<<std::endl;
		std::cout<<"///////////////////////////////////////////////////////////////\n";

				if (stepnumber_t == 0) {
					int i;
			}
	}
}

bool written = false;
//reset function (called after each trial)
void NimmExp1::reset_params()
{
	xt_ico_lowpass = 0;
	xt_ico_lowpass2= 0;
	xt_ico_lowpassold = 0;
	xt_ico_lowpass2old = 0;
	xt_ico_lowpass3old = 0;
	irl_lowpass = 0;
	irr_lowpass = 0;
	lowpass_dis_x = 0;
	lowpass_dis_y = 0;
	exploration_old = 0;

	exploration_lowpass_g[0] = 0;

	for(int i =0; i< UDIM /*2*/; i++) {
		exploration_lowpass_g[i] = 0;
	}

	for (int i=0; i< XDIM; i++)
		input_old[i] = 0;

	for (int i=0; i< NUMBER_OF_BALLS; i++) {
		taken_rewards[i] = 0;
		reached_goals[i] = 0;
	}


	if (robotstate.reset_robot_state() && (!written)) {
		written = true;
		if (WRITE_RBF_TO_FILE) {
			std::cout<<"Writing RBF to File\n"<<std::endl;
			VALUE.write_to_file(rbf_num_units, rbf_num_IN, rbf_num_out);
		}
		if (TERMINATE_AFTER_LEARNING)
			terminate_immediatly = true;
	}


	if (!USE_SAVED_RBF) {
		for (int i=0; i<this->samples.size(); i++) {
				update_critic_actor( samples.at(i).inputs.data(),
								samples.at(i).tds.at(0),
								rate_valuefunction,
								samples.at(i).p_error,
								rate_policy,
								samples.at(i).tds.at(1)
									);

			}
	}

	samples.clear();


	if (WRITE_CRITIC_ACTOR_WEIGHTS_TO_TEXT_FILE)
				VALUE.print_weights("Weights", rbf_num_units, rbf_num_out);

	std::cout<<"epoch number"<<ntrial<<std::endl;


	std::cout<<"terminate_immediatly"<<terminate_immediatly<<std::endl;

}
//save input signals' values to a file
void NimmExp1::save_inputs()
{
	ofstream _file;
	_file.open("inputs_log", std::fstream::app);
	_file<<xt[0]<<" "<<xt[_ias1]<<" "<<xt[_ias2]<<" "<<xt[3]<<"\n";
	_file.close();

}
//save output signals' values to a file
void NimmExp1::save_output()
{
	ofstream _file;
	_file.open("output_log", std::fstream::app);
	_file<<actual_out<<"\n";
	_file.close();
}
//save old input signals
void NimmExp1::copy_input(double* inputs, double* inputs_old)
{
	for (int i=0; i<XDIM; i++) {
		inputs_old[i] = inputs[i];
	}
}
//step function (called for each time step)
void NimmExp1::step(const sensor* x_, int number_sensors, motor* y_, int number_motors)
{
	int controller_step = 1;
	calculateAnglePositionFromSensors(x_);
	calculateDistanceToGoals(x_);
	get_ir_signals(x_);
	irl_back = x_[8];
	irr_back = x_[9];
	inputSensorSignals(x_);


	double* dummy;
	//if ((stepnumber_t % controller_step == 0) || (reward_function(xt, dummy) != 0)) {
		controller_core(x_, number_sensors, y_, number_motors);
	//}


	stepnumber_t++;
}
//Controller step function (called for each time step)
void NimmExp1::controller_core(const sensor* x_, int number_sensors, motor* y_, int number_motors)
{
	//std::cout<<"step:"<<stepnumber_t<<std::endl;
	double scale = 0.5;// 1.0;

	bool manual_control = false;
	bool actor_critic_control =  true;

	// here begins actor critic learning
	//////////////////////////////////////////////////////Define section//////////////////////////////////
	static int iter = 0; // Initial only one time
	static int iter_old = 0;
	static int trial_n = 0;
	static double Vt_old = 0.0;// Initial only one time
	static double rt_old = 0.0;
	//td_error = 0.0;
	double  rt_reducenoise;
	//----AC network parameters------------//

	iter_old = iter;


	/********************************************************SENSORY INPUTS*******************/
	xt[0] =xt_ico_lowpass;
	xt[1] = xt_ico_lowpass2;
	xt[2] = xt_ico_lowpass3;
	xt[3] = robotstate.get_state_array();


	 reduce_with_time = exp(-(reduce_with_time_counter)/500.0);

	 exp_output_decay = exp(-reduce_noise/200.0);//100);

	/*---(4) Check state ---------------------------*/
	failure_flag = check_limit(xt);

	/*if (write_to_file) {
		save_inputs();
		save_output();

	}*/



	/*************************************************************AFTER Fail->RESET*******************/
	if(resetLearning_RL == 0)
	{
		stepnumber_t = 0;
		++current_epoch;
		if (hit_the_goal && (current_epoch>200))
			reduce_with_time_counter++;

		failure_flag = 0;

		//////////////////////////////added
		ntrial++;
		//robotstate.set_state(ntrial);
		reset_params();
		//Vt_old = value_function(xt/*inputs*/); //Replaced by ESN // RESET p_old or reset Vt_old = V1 // as expected Max
		//Vt = value_function(xt/*inputs*/); //Replaced by ESN // RESET p_old or reset Vt_old = V1 // as expected Max
		rt_old = 0;
		//////////////////////////////////////////////////////////

		iter = 0;
		acum_reward = 0.0; // RESET acc reward // may need to analyze or evaluate system
		sig_out_log[ntrial] = sig_out[0]; // may need to analyze or evaluate system
		resetLearning_RL++;
	}


	/*---(3) Cal reward-----------------------------*/
	rt = reward_function(xt, ut);


	/*---(1) Output from current policy--------------*/
	old_ut = ut[0];
	//output_policy(xt /*in*/, ut /*steering control, return*/);


	double out [2];
	get_RBF_network_out(xt, out);
	Vt = out[0];
	ut[0] = out[1];
	//ut[0] = clip(ut[0], -1.0, 1.0); /// CLIP limit ut0


	/*---(5) Value function estimation--------------*/
	//Vt = value_function(xt/*inputs*/); //Replaced by ESN

	real_v = Vt / 100.0;
	//acum_reward += rt;
	//acum_reward += pow(gam,iter)*rt;

	/********************************************************************* LEARNING STATE*******************/
	if((ntrial < MAX_TRIAL)&&(!failure_flag))
	{
		/*---(2) Move robot------------------------------*/
		y_ac[0] =  scale*1+ut[0];//ut[0]; // left front wheel
		y_ac[1] = scale*1-ut[0];//ut[1]; // right front wheel
		y_ac[2] = scale*1+ut[0];//ut[0]; // left rear wheel
		y_ac[3] = scale*1-ut[0];//ut[1]; // right rear wheel

		//////////////////////////////////////////////////////////////TD calculation/////////////////////
		td_error = rt + gam*Vt - Vt_old;

		/*---(9) Update parameters for value function approximator weights-*/
		//update_valuefunction_trace(input_old); //return --> curr->dw = etrace of all softmax RBF hidden neurons (curr->softac);
		//update_valuefunction(input_old/*input*/, td_error/*TDerror*/,rate_valuefunction);



		p_error = rt + Vt - Vt_old;

		submit_sample(input_old, td_error, p_error, exploration_old);

		/*---(10) Update parameters for policy weights-------------*/

		//if ((p_error > 0)
				//&& (ntrial > 30)
		//		)
		//{
			//update_policy_trace(input_old, lambda_p);
			//update_policy_actor(1, rate_policy, input_old); // Actor-critic learning
		//}



		if(resetLearning_RL == 1)
			checkreset = 10;
		else
			checkreset = 0;

		iter++;
		resetLearning_RL++;
	}

	if (!robotstate.Final_goal_is_reached()) {
		for (int i=0; i<4; i++)
			y_[i] = y_ac[i];
		/*y_[0] = y_ac[0]; // left front wheel
		y_[1] =	y_ac[1]; // right front wheel
		y_[2] =	y_ac[2]; // left rear wheel
		y_[3] =	y_ac[3]; // right rear wheel*/
	} else {
		for (int i=0; i<4; i++)
			y_[i] = 0;

	}

	////save for next iteration
	copy_input(xt, input_old);
	exploration_old = actual_out - actor_out;//exploration_out; exp = y - a depends on your exploration generation strategy ;)
	Vt_old = Vt;
	rt_old = rt;


	// manual steering
	if(manual_control) {
		for (int i=0;i<4;i++)
			y_[i]=mc[i];
	}

	if (terminate_after_this_step) {
		terminate_after_this_step = false;
		failure_flag = true;
	}
}
//help function to aid printing function
int get_number_of_digits(int n)
{
    int count = 0;
    while(n > 0)
    {
        n  /= 10;
        count++;
    }
    return count;
}

#include <iostream>
#include <string.h>
void NimmExp1::stepNoLearning(const sensor* x_, int number_sensors,
		motor* y_, int number_motors)
{

}
double NimmExp1::sigmoid(double num)
{
	return 1./(1.+exp(-num));
}
double NimmExp1::tanh(double num)
{
	return 2./(1.+exp(-2*num))-1.;
}
//reward function
double NimmExp1::reward_function(double *x /*state*/, double *u /*action*/)
{
	bool far = true;
	for (int i=0; i<NUMBER_OF_BALLS; i++)
		if (distances[i] < 0.02)
			far = false;

	if ((irl_lowpass > 0.07) || (irr_lowpass > 0.07))
		return -1;

	double state_reward = robotstate.get_reward(distances);

	return state_reward;
}
//check the termination of the current iteration
int NimmExp1::check_limit(double *x)//, double irr_back, double irl_back)
{

	int flag=0;


	bool dis_validation = true;
	for (int i=0; i<NUMBER_OF_BALLS; i++)
		if (distances[i] < 0.02)
			dis_validation = false;

	//if ((input_distance_s > 0.02)&&(input_distance_s2 > 0.02)&&(input_distance_s3 > 0.02))
	if (dis_validation)
	//if( xt_ico2[_ias1] < 0.03 || xt_ico3[_ias1] < 0.03 || irl_lowpass*1.5 > 0.2 || irr_lowpass*1.5 > 0.2)
	if( //xt_ico[_ias1] < -1.97 ||  xt_ico2[_ias1] < -1.97 ||
			irl_lowpass > 0.8 || irr_lowpass > 0.8)
	{
		flag = 1;

	}

	if (robotstate.Final_goal_is_reached())
		flag = 1;


	if (stepnumber_t > 10000)
		flag = 1;



	return flag;


}
//	Initial RBF network
void NimmExp1::init_funcapprox()
{

	int num_unit_1 = 5;
	int num_unit_2 = 5;

	double max_range1 = +M_PI;
	double min_range1 = -M_PI;

	double max_range2 = 2;
	double min_range2 = 0;


	int i;
	int state_index[XDIM /*input, e.g., 4*/];
	double xc[XDIM /*input, e.g.,4*/];

	ngnet->init_incsbox(&VALUE, 4, 2);
	ngnet->reset_incsbox(&VALUE);


	//***Set it differently according to a number of your inputs, here only 2 inputs
	basis[0] = num_unit_1; // N_IN0_RBF/*3*/; // Number of RBF
	basis[1] = num_unit_1; // N_IN1_RBF/*3*/; // Number of RBF
	basis[2] = num_unit_1;
	basis[3] = num_unit_2;


	xmax[_ias0] = max_range1;
	xmin[_ias0] = min_range1;

	xmax[_ias1] = max_range1;
	xmin[_ias1] = min_range1;

	xmax[_ias2] = max_range1;
	xmin[_ias2] = min_range1;

	xmax[3] = max_range2;
	xmin[3] = min_range2;

	//2) find width of each input
	for(i=0;i<xdim /*4 inputs*/;i++)
	{
		basis_width[i]  = (xmax[i] - xmin[i])/(2.0*basis[i]/*number of centers*/); // variance (the measure of the width of the distribution)
		wbasis[i] = 1.0/basis_width[i]; //width of each input
	}


	nbasis = 0; // reset

	//--------- two inputs-----------------------//
	for(state_index[0]=0; state_index[0]<basis[0] /*3*/; state_index[0]++) // angle left
	for(state_index[1]=0; state_index[1]<basis[1] /*3*/; state_index[1]++) // angle right
	for(state_index[2]=0; state_index[2]<basis[2] /*3*/; state_index[2]++) // angle right
	for(state_index[3]=0; state_index[3]<basis[3] /*3*/; state_index[3]++) // angle right
	{
		for(i=0;i<xdim ;i++)
			xc[i]  = (xmin[i]+basis_width[i])+state_index[i]*2.0*basis_width[i];
		ngnet->put_incsbox(&VALUE, 4 , 2, xc, wbasis, &nbasis);
	}
}
//get the next action and the V value
void NimmExp1::get_RBF_network_out(double* in, double* out) {
	double Value [2];
	//std::cout<<Value[0]<<std::endl;
	ngnet->incsbox_output(&VALUE, in, Value, &nbasis);
	out[0] = Value[0];

	if (!USE_SAVED_RBF) {
		//get noise
	#define		exploration_gain 0.99//0.9997
		double exp = robotstate.get_exploration(real_v, exploration_gain);
		//exp = clip(exp, -0.3, 0.3);
		//Value[1] = clip(Value[1], -0.3, 0.3);
		exploration_out = exp;

		out[1] = robotstate.bind_exploration(Value[1], exploration_out);

		actor_out = Value[1];
		actual_out = out[1];
	} else {
		exploration_out = 0;
		actor_out = Value[1];
		actual_out = actor_out;
		out[1] = actual_out;
	}
}
//function to add a sample (consists from the current step's values) to the learning samples list
void NimmExp1::submit_sample(double* input_old, double td_error, double pure_error, double actor_update_value) {
	Sample s;
	for (int i=0; i<XDIM; i++)
		s.inputs.push_back(input_old[i]);
	s.tds.push_back(td_error);
	s.tds.push_back(actor_update_value);
	s.p_error = pure_error;
	samples.push_back(s);
}
//perform the updates to the RBF network (actor + critic)
void NimmExp1::update_critic_actor(double* input_old, double td_error
			, double rate_valuefunction
			, double p_error
			, double rate_policy
			, double actor_update_value
			) {

	ngnet->incsbox_trace(&VALUE, input_old, 0, &nbasis);

	bool update_actor = false;
	if (p_error > 0)
		update_actor = true;

	double td_error_temp = td_error;

	ngnet->incsbox_update_v_action_pairs(
			&VALUE, input_old, actor_update_value, td_error_temp,
				rate_valuefunction, rate_policy, &nbasis,
				wbasis ,
				error_thresh_critic ,
				unit_activation_thresh_critic, update_actor);
}
