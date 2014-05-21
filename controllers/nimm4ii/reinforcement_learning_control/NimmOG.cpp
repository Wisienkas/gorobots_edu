/*************************************************************************
 * Reinforcement learning controller for nimm robot in
 * one goal scenario
 * Created/modified by Bassel Zeidan Jan 6, 2013
 **************************************************************************/
//General information
/*************************************************************************
 * Note: 1) one RBF network (with two outputs) is used for the actor and the critic to speed up the execution.
 * 		 However, the learning rule is different for these two outputs.
 *		 2) In each learning iteration, the learning samples (TD error, inputs, outputs) for each time step
 *		 	are saved in a list called (samples) until the termination of the iteration and then the updates
 *		 	are done over the RBF network (learning in a batch mode)
 *		 	submit_sample() ---> function to save a learning sample (called at each time step)
 *		 	conduct_updates() ---> perform updates over the RBF network (called after the learning iteration is terminated)
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
 * 		1 when the robot reaches the goal.
 * 		-1  when the robot hits an obstacle
 *
 * Parameters:
 * 		The actor's learning rate = 0.01
 * 		The critic's learning rate = 0.05
 * 		The discount factor = 0.999
 * 		exploration scale factor = 1.1
 *
 *
 *
 *
 * Actor:
 * 		Number of inputs: 3
 * 		inputs: the relative angle from three transmitters (spheres) T1 blue, T2 Green, T3 yellow
 * 		Number of hidden neurons: 1000
 * 		Number of outputs: one control output. (the second output of the RBF network)
 *
 * Critic:
 * 		Number of inputs: 3
 * 		inputs: the relative angle from three transmitters (spheres) T1 blue, T2 Green, T3 yellow
 * 		Number of hidden neurons: 1000
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
 * * WRITE_AVERAGE_OF_EPOCHS (1) write the number of trials that are needed before convergence (to the average file)
 *   (0) Otherwise
 * * WRITE_INPUT_OUTPUT_SIGNALS (1) write input - output signals to the file for each time step (0) Otherwise
 *************************************************************************************/
#define WRITE_RBF_TO_FILE 0
#define USE_SAVED_RBF 1
#define WRITE_CRITIC_ACTOR_WEIGHTS_TO_TEXT_FILE 0
#define TERMINATE_AFTER_LEARNING 0
#define WRITE_AVERAGE_OF_EPOCHS 0
#define WRITE_INPUT_OUTPUT_SIGNALS 0

using namespace std;

#include "NimmOG.h"
#include <selforg/controller_misc.h>
#include "utils/rbf-framework/ngnet.h"

#define rbf_num_units 1000
#define rbf_num_IN 3
#define rbf_num_out 2


#ifndef clip
#define	clip(x/*input*/,l /*lower limit*/,u /*upper limit*/)( ((x)<(l)) ? (l) : ((x)>(u)) ? (u) : (x))
#endif

#ifndef min
#define min(x/*e.g., 1*/, y) ((x <= y) ? x : y)
#endif

#ifndef max
#define max(x/*e.g., 0*/, y) ((x >= y) ? x : y)
#endif


namespace {
  NGNet* ngnet;

  //----AC network parameters------------//
  int current_epoch;

  Cell VALUE(rbf_num_units, rbf_num_IN, rbf_num_out);

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

  bool written = false;
}

//controller constructor
NimmOG::NimmOG(const NimmOgConf& _conf)
: AbstractController("NimmOG", "$Id: "), conf(_conf)
{
	//added:

	td_error=0.0;
	current_epoch= 0;
	ngnet = new NGNet(rbf_num_IN, rbf_num_out);




	preprogrammed_steering_control_active =false;
	t=0;
	//for manual steering
	mc[0]=0;
	mc[1]=0;
	mc[2]=0;
	mc[3]=0;

	//-----AC_ICO Learning------------------//
	resetLearning_RL = 0;
	output_actor = 0.0;

	//-----Set vector size------------------//
	input_angle_s.resize(NUMBER_OF_BALLS);
	ias_w.resize(2);
	etrace_io.resize(2);
	etrace_io_old.resize(2);


	//-----Plot values----------------------//

	addInspectableValue("VT",&Vt,"Vt");
	addInspectableValue("rt",&rt,"rt");
	addInspectableValue("TD_Error",&td_error,"TD_Error");
	addInspectableValue("exp_output_decay",&exp_output_decay,"exp_output_decay");
	addInspectableValue("verror",&verror,"verror");
	addInspectableValue("exploration_out",&exploration_out,"exploration_out");
	addInspectableValue("actual_out",&actual_out,"actual_out");
	addInspectableValue("actor_out",&actor_out,"actor_out");
	addInspectableValue("irl",&irl_lowpass,"irl");
	addInspectableValue("irr",&irr_lowpass,"irr");
	addInspectableValue("xt[_ias0]",&xt[_ias0],"xt[_ias0]");
	addInspectableValue("xt[_ias1]",&xt[_ias1],"xt[_ias1]");
	addInspectableValue("xt[_ias2]",&xt[_ias2],"xt[_ias2]");
	addInspectableValue("distances[0]",&distances[0],"distances[0]");

	initialized = false;
	use_Learned_weightsv2 = false;
	large_input_rangev2 = false;
	no_learning_noise = true;

};
//controller destructor
NimmOG::~NimmOG()
{
}
//initialization function
void NimmOG::init(int sensornumber, int motornumber, RandGen* randGen)
{
	stepnumber_t = 0;
	++current_epoch;
	if(!initialized) //-----------------------------------------------(FRANK)
	{

		w_ico = 0.5;
		w_ac= 0.5;
		rate = 0.0005;
		ss_rate = 0.0005;

		learn_combined_weights = true;

		scenario_flag = 1;

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

		nbasis_value =0;
		xdim = XDIM;
		udim = 2;

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
		gam = 0.999;//0.99999999999;//0.98;//0.98; // discount factor of Vt (critic)
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


		for (int i=0; i<NUMBER_OF_BALLS; i++){
				low_pass_angles_old[i] = 0;
				low_pass_angles[i] = 0;
			}

		termination_counter = 0;
		terminate_now = false;
		absorbed_reward = 0;
	}
}
//calculate relative angles
void NimmOG::calculateAnglePositionFromSensors(const sensor* x_) //as it says on the tin
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
		i+=3;
	}
}
//calculate distances
void NimmOG::calculateDistanceToGoals(const sensor* x_)
{

	double distance_scale = 0.001;//1/100;

	for (int i=0; i<NUMBER_OF_BALLS; i++){
		max_dis = position_x_robot*position_x_robot; //normalize
		distance = (sqrt(pow(x_[13+(i*3) /*11 y*/],2)+pow(x_[12+(i*3)/*10 x*/],2)));
		distances[i] = distance*distance_scale;//85;//max_dis;
	}
}
//low pass filters for input signals
void NimmOG::inputSensorSignals(const sensor* x_)
{
	xt_ir = input_ir_s;


	double gain_r = 0.9;

	for (int i=0; i<NUMBER_OF_BALLS; i++) {
		low_pass_angles_old[i] = low_pass_angles[i];
		low_pass_angles[i] = ((1-gain_r)*input_angle_s.at(i)) + (low_pass_angles_old[i]*gain_r);//red
	}
}
//low pass filter for IR signals
void NimmOG::get_ir_signals(const sensor* x_)
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
void NimmOG::print_to_cout(int steps_period)
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
				 <<"nbasis -------> "<<nbasis_value<<std::endl
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
//reset function (called after each trial)
void NimmOG::reset_params()
{
	xt_ico_lowpass = 0;
	xt_ico_lowpass2= 0;
	irl_lowpass = 0;
	irr_lowpass = 0;
	lowpass_dis_x = 0;
	lowpass_dis_y = 0;
	exploration_old = 0;

	absorbed_reward = 0;

	for(int i =0; i< UDIM /*2*/; i++) {
		exploration_lowpass_g[i] = gauss()/30.0;
	}

	for (int i=0; i< XDIM; i++)
		input_old[i] = 0;

	//ngnet->reset_incsbox(&VALUE);
	//ngnet_actor->reset_incsbox(&VALUE_ACTOR);
	conduct_updates();
	if (WRITE_CRITIC_ACTOR_WEIGHTS_TO_TEXT_FILE)
			VALUE.print_weights("Weights", rbf_num_units, rbf_num_out);

	//terminate_now = true;
}
//save old input signals
void NimmOG::copy_input(double* inputs, double* inputs_old)
{
	for (int i=0; i<XDIM; i++) {
		inputs_old[i] = inputs[i];
	}
}
//save input signals' values to a file
void NimmOG::save_inputs()
{
	ofstream _file;
	_file.open("inputs_log", std::fstream::app);
	_file<<xt[0]<<" "<<xt[_ias1]<<" "<<xt[_ias2]<<"\n";
	_file.close();

}
//save output signals' values to a file
void NimmOG::save_output()
{
	ofstream _file;
	_file.open("output_log", std::fstream::app);
	_file<<actual_out<<"\n";
	_file.close();
}
//step function (called for each time step)
void NimmOG::step(const sensor* x_, int number_sensors, motor* y_, int number_motors)
{
	/////print some stuff
	//print_to_cout(1000);


	//---1) Calculating angle position from goals
	calculateAnglePositionFromSensors(x_);

	//---3) Calculating distance to the goals 1-4
	calculateDistanceToGoals(x_);

	//get ir signals
	get_ir_signals(x_);

	irl_back = x_[8];
	irr_back = x_[9];

	inputSensorSignals(x_);

	static int iter = 0; // Initial only one time
	static int iter_old = 0;
	static int trial_n = 0;
	static double Vt_old = 0.0;// Initial only one time
	static double rt_old = 0.0;
	//td_error = 0.0;
	double  rt_reducenoise;
	//----AC network parameters------------//

	iter_old = iter;

	/*****SENSORY INPUTS*******************/
	xt[_ias0] =low_pass_angles[1];
	xt[_ias1] =low_pass_angles[2];
	xt[_ias2] = low_pass_angles[3];

	if (WRITE_INPUT_OUTPUT_SIGNALS) {
		save_inputs();
		save_output();
	}
	//std::cout<<"Epoch = "<<current_epoch<<std::endl;

	failure_flag = check_limit(xt);

	/*****AFTER Fail->RESET*******************/
	if(resetLearning_RL == 0 /*when approach goal or hit obstacles*/)
	{
		stepnumber_t = 0;
		++current_epoch;
		if (hit_the_goal && (current_epoch>200))
			reduce_with_time_counter++;
		//----Important for learning-----//
		//Robot will be reset after this by simulation//
		failure_flag = 0;


		//////////////////////////////added
		reset_params();
		//Vt_old = value_function(xt/*inputs*/); //Replaced by ESN // RESET p_old or reset Vt_old = V1 // as expected Max
		//Vt = value_function(xt/*inputs*/); //Replaced by ESN // RESET p_old or reset Vt_old = V1 // as expected Max
		rt_old = 0;
		//////////////////////////////////////////////////////////




		//----Important for learning-----//

		ntrial++;
		iter = 0;
		acum_reward = 0.0; // RESET acc reward // may need to analyze or evaluate system
		sig_out_log[ntrial] = sig_out[0]; // may need to analyze or evaluate system
		resetLearning_RL++;
		std::cout<<"epoch"<<current_epoch<<std::endl;
	}


	/*---(3) Cal reward-----------------------------*/
	rt = reward_function(xt, ut);


	double net_out [2];
	//get action and v value
	get_v_action_pairs(xt, net_out);
	//for (int i=0; i<2; i++)
	//	std::cout<<net_out[i]<<std::endl;

	output_policy(net_out /*in*/, ut /*steering control, return*/);
	//ut[0] = clip(ut[0], -1.0, 1.0); /// CLIP limit ut0

	//get v value
	//Vt = value_function(xt/*inputs*/); //Replaced by ESN
	Vt = net_out[0];

	//////exploration
	rt_reducenoise = rt;
	if(rt_reducenoise < 0)
		reduce_noise -= 3;
	else if(rt_reducenoise > 0.0)
		{
			reduce_noise += 3;
			//hit_the_goal = true;
		}

	if(reduce_noise<0)
		reduce_noise = 0;
	//ADD

	exp_output_decay = exp(-reduce_noise/200.0);//100);
	////////////




	/*** LEARNING STATE*******************/
	if ((ntrial < MAX_TRIAL) &&(!failure_flag))
	{
		/*---(1) Output from current policy--------------*/
		old_ut = ut[0];
		double scale = 0.5;
		/*---(2) Move robot------------------------------*/
		y_ac[0] = scale*1+ut[0];//ut[0]; // left front wheel
		y_ac[1] = scale*1-ut[0];//ut[1]; // right front wheel
		y_ac[2] = scale*1+ut[0];//ut[0]; // left rear wheel
		y_ac[3] = scale*1-ut[0];//ut[1]; // right rear wheel


		/*--- TD error-------------------------------*/
		td_error = rt_old + (gam*Vt) - Vt_old;


		//update_valuefunction_trace(input_old); //return --> curr->dw = etrace of all softmax RBF hidden neurons (curr->softac);
		//update_valuefunction(xt/*input*/, td_error/*TDerror*/,rate_valuefunction);

		/*---(10) Update parameters for policy weights-------------*/
		verror = rt_old + Vt - Vt_old;
		//if (verror > 0)
		//{
			//update_policy_trace(input_old, lambda_p);
			//update_policy_actor(1, rate_policy, input_old); // Actor-critic learning
		//}

		submit_sample(input_old, td_error, exploration_old, verror);

		if(resetLearning_RL == 1)
			checkreset = 10;
		else
			checkreset = 0;
		iter++;
		resetLearning_RL++;
	}


	y_[0] = y_ac[0]; // left front wheel
	y_[1] =	y_ac[1]; // right front wheel
	y_[2] =	y_ac[2]; // left rear wheel
	y_[3] =	y_ac[3]; // right rear wheel


	////save for next iteration
	copy_input(xt, input_old);
	exploration_old = actual_out - actor_out;
	Vt_old = Vt;
	rt_old = rt;



	//delete[] net_out;
	stepnumber_t++;
}

#include <iostream>
#include <string.h>

void NimmOG::stepNoLearning(const sensor* x_, int number_sensors,
		motor* y_, int number_motors)
{

}

double NimmOG::sigmoid(double num)
{
	return 1./(1.+exp(-num));
}

double NimmOG::tanh(double num)
{
	return 2./(1.+exp(-2*num))-1.;
}
//exploration function
double NimmOG::get_exploration(double *net_out) {
//net_out v + action
	double V1, V0;
	double y;
	double vt_value = net_out[0];///100.0;
	double max_value, min_value;
	double sig0_nolearning;
	double sig_nolearning;

	V1 = 0; //50.0;//1.0;//50.0;//50.0;//1.0;//50.0;//50.0;//50.0;//70.0;//50.0;//1.0;//50.0;//50.0;//1.0; // Max expected value//---------------------------------**** Change
	V0 = -5; //-1.0;//0.0;//0;//0.0;//-125;//0.5; // Min expected value//---------------------------------****  Change


	y = (V1-vt_value)/(V1-V0);
	max_value = max(0, y);
	min_value = min(/*0.1*/1 ,max_value); //changing from 1 to 0.1



	sig0_nolearning = 1.1;//5.0;//si;

	sig_nolearning = sig0_nolearning*min_value;

	double lp_gain =  0.998;//*(1 - min_value);//0.95;

	exploration_g[0] = gauss();
	exploration_lowpass_old_g[0] = exploration_lowpass_g[0];
	exploration_lowpass_g[0] = exploration_lowpass_old_g[0]*lp_gain+((1-lp_gain)*exploration_g[0]);

	return exploration_lowpass_g[0]*sig_nolearning;
}
//function to produce the next action
void NimmOG::output_policy(double *net_out, double *u)
{
	if (!USE_SAVED_RBF) {

			double exp = get_exploration(net_out)*exp_output_decay;
			exp = clip(exp, -0.1, 0.1);
			net_out[1] = clip(net_out[1], -0.1, 0.1);

			//net_out[1] = 0;
			//exp = 0;

			exploration_out = exp;

			actor_out = net_out[1];

			if (exp_output_decay >= 0.0001) {
				termination_counter = 0;

				if (current_epoch % 2 == 0)
					u[0] = actor_out + exploration_out;
				else u[0] = exploration_out;
			} else {

				if (last_epoch != current_epoch) {
					last_epoch = current_epoch;

					termination_counter++;
				}

				u[0] = actor_out;
			}

			actual_out = u[0];
	} else {
		exploration_out = 0;
		actor_out = net_out[1];
		actual_out = actor_out;
		u[0] = actual_out;
	}
}
/*************************************************************
 *	Gaussian random variable: just a sum of uniform distribution
 *	Average = 0, Variance = 1, (1.2)
 *************************************************************/
double NimmOG::gauss()
{
	double	sum;
	int 	i;

	for( sum = i = 0; i < 12; i++)
		sum += (1.0*(double)rand()/(RAND_MAX));
	return(sum - 6.0);
}
//reward function [-1, 1]
double NimmOG::reward_function(double *x /*state*/, double *u /*action*/)
{

	double r_total = 0.0;


	if (distances[0] < 0.02) {
		absorbed_reward++;
		return 1;
	}


	if ((irl_lowpass > 0.07) || (irr_lowpass > 0.07))
		//r_total = -1;
		return -1;

	return r_total;
}

//called to terminate the current epoch according to certain conditions
int NimmOG::check_limit(double *x)//, double irr_back, double irl_back)
{

	int flag=0;

	if (absorbed_reward > 10000)
		flag = 1;

	if (termination_counter > 15) {
		if (!written) {
			written = true;
			if (WRITE_AVERAGE_OF_EPOCHS) {
				std::ofstream myfile;
				myfile.open ("Results.txt", std::ios::app);
				myfile<<"Trials number = "<<current_epoch<<std::endl;
				myfile.close();
			}
			flag = 1;
			if (WRITE_RBF_TO_FILE) {
				std::cout<<"Writing RBF to File\n"<<std::endl;
				VALUE.write_to_file(rbf_num_units, rbf_num_IN, rbf_num_out);
			}
			if (TERMINATE_AFTER_LEARNING)
				terminate_now = true;
		}
	}

	//if( xt_ico2[_ias1] < 0.03 || xt_ico3[_ias1] < 0.03 || irl_lowpass*1.5 > 0.2 || irr_lowpass*1.5 > 0.2)
	if( irl_lowpass > 0.8 || irr_lowpass > 0.8)
	{
		flag = 1;

	}

	if (current_epoch > 10000)
		terminate_now = true;

	return flag;


}
//initalization function for the RBF networks
void NimmOG::init_funcapprox()
{

	int i;
	int state_index[XDIM /*input, e.g., 4*/];
	double xc[XDIM /*input, e.g.,4*/];

	ngnet->init_incsbox(&VALUE,XDIM , 2 /*1 one output*/);
	ngnet->reset_incsbox(&VALUE);

	//***Set it differently according to a number of your inputs, here only 2 inputs
	basis[_ias0] = 10; // N_IN0_RBF/*3*/; // Number of RBF
	basis[_ias1] = 10; // N_IN1_RBF/*3*/; // Number of RBF
	basis[_ias2] = 10;


	//***Set it differently according to a number of your inputs, here only 3 inputs
	//1) Input limitation MAX MIN
	xmax[_ias0] = MAX_IN0;
	xmin[_ias0] = MIN_IN0;

	xmax[_ias1] = MAX_IN1;
	xmin[_ias1] = MIN_IN1;

	xmax[_ias2] = MAX_IN2;
	xmin[_ias2] = MIN_IN2;

	//2) find width of each input
	nbasis = 0; // reset
	for(i=0;i<xdim /*4 inputs*/;i++)
	{
		basis_width[i]  = (xmax[i] - xmin[i])/(2.0*basis[i]/*number of centers*/); // variance (the measure of the width of the distribution)
		wbasis[i] = 1.0/basis_width[i]; //width of each input
	}

	for(state_index[_ias0]=0; state_index[_ias0]<basis[_ias0] /*3*/; state_index[_ias0]++) // angle left
	for(state_index[_ias1]=0; state_index[_ias1]<basis[_ias1] /*3*/; state_index[_ias1]++) // angle right
	for(state_index[_ias2]=0; state_index[_ias2]<basis[_ias2] /*3*/; state_index[_ias2]++) // angle right
	//for(state_index[_ias3]=0; state_index[_ias3]<basis[_ias3] /*3*/; state_index[_ias3]++) // angle right
	{
		for(i=0;i<xdim /*2 inputs*/;i++)
			xc[i]  = (xmin[i]+basis_width[i])+state_index[i]*2.0*basis_width[i];
		ngnet->put_incsbox(&VALUE, XDIM , 2, xc, wbasis/*1/Variance*/, &nbasis);
	}


}
//get the current action and the current V value
void NimmOG::get_v_action_pairs(double* in, double* out) {
	ngnet->incsbox_output(&VALUE, in, out, &nbasis);
}
//function to add a sample (consists from the current step's values) to the learning samples list
void NimmOG::submit_sample(double* in, double td_v, double td_actor, double perror) {
	Sample s;
	s.inputs.push_back(in[0]);
	s.inputs.push_back(in[1]);
	s.inputs.push_back(in[2]);
	s.tds.push_back(td_v);
	s.tds.push_back(td_actor);
	s.perror = perror;
	this->samples.push_back(s);
}
//perform the updates to the RBF networks (actor + critic)
void NimmOG::conduct_updates() {
	if (!USE_SAVED_RBF) {
		for (int i=0; i<this->samples.size(); i++) {
			update_rbf_network(this->samples[i].inputs.data(),
			this->samples[i].tds[0], this->samples[i].tds[1], rate_valuefunction, this->samples[i].perror, rate_policy);
		}
		this->samples.clear();
	}
}
//RBFs update function
void NimmOG::update_rbf_network(double* input_old, double td_error, double td_actor, double rate_valuefunction, double p_error, double rate_policy) {

	double td = td_error;

	//output_elig[0] = output_grad[0];
	ngnet->incsbox_trace(&VALUE, input_old, lambda_v , &nbasis);

	bool update_actor;
	if (p_error > 0)
		update_actor = true;
	else update_actor = false;

	ngnet->incsbox_update_v_action_pairs(&VALUE, input_old, td_actor, td, rate_valuefunction, rate_policy, &nbasis, wbasis
			, error_thresh_critic, unit_activation_thresh_critic, update_actor);
}

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////removed functions but can be activated if needed////////
////////////////////////////these function can be used for incremental RL mode//////
void NimmOG::update_policy_actor(double td_error, double rate, double* input)
{
	int i;
	double td_temp = -td_error; //because of the RBF lib

	//ngnet_actor->incsbox_update_actor(&VALUE_ACTOR, input, output_elig, &td_temp, rate /*e.g., 0.5*/, &nbasis_value
			///*number of hidden neurons e.g., initial 27*/,
		//	wbasis_value /*varience of input0, input1*/,
		//	error_thresh_actor /*e.g. done 0.01,    0.1*/,
		//	unit_activation_thresh_actor /*e.g.,done 0.03          0.6*/);

//	sig_out[0] += rate*td_error*sig_elig[0];
	//sig_out[1] += rate*td_error*sig_elig[1];
	//sig_out[2] += rate*td_error*sig_elig[2];
	//sig_out[3] += rate*td_error*sig_elig[3];
	//printf("sig_elig[0] = %f : sig_out = %f Dtd_error = %f td_error = %f\n", sig_elig[0],sig_out[0],td_error-td_error_old, td_error);
}

void NimmOG::update_valuefunction_trace(double *x)
{
	ngnet->incsbox_trace(&VALUE, x, lambda_v /*trace, set to 0.0 = no trace*/, &nbasis /*number of hidden neurons*/);

}

void NimmOG::update_valuefunction(double *x, double td/*TDerror*/, double rate)
{

	//CHANGE
	double td_error_tmp[1];
	double vt, yt[1];
	td_error_tmp[0] = -td;

	//	double td_error_tmp[1], vt, yt[1];
	//		td_error_tmp[0] = -td;


	//x = inputs -> change over time
	//td_error_tmp = -TDerror-> change over time and go to zero when learning complete
	//nbais = adaptive critic will increase from initial, e.g., 9 to nbasis>9
	//wbasis /*varience of input0, input1*/ = fixed
	//error_thresh = fixed /*e.g. 0.1*/
	//unit_activation_thresh = fixed /*e.g., 0.6*/
	ngnet->incsbox_update(&VALUE, x,
			td_error_tmp,
			rate /*e.g., 0.5*/,
			&nbasis /*number of hidden neurons e.g., initial 27*/,
			wbasis /*varience of input0, input1*/,
			error_thresh_critic /*e.g. 0.1*/,
			unit_activation_thresh_critic /*e.g., 0.6*/);
}

void NimmOG::print_RBF_network_to_file(int epoch)
{
	bool write_to_file = false;
	bool write_statistics = false;
	bool write_critic = false;
	bool write_actor = false;


	if (write_to_file)
	{
		int Max_digits = 6;
		int number_digits = get_number_of_digits(epoch);
		int dif = Max_digits - number_digits;
		std::string critic_file;
		std::string actor_file;
		std::string statistics_file;

		critic_file.append("RBFTEST//RBFData_critic");
		actor_file.append("RBFTEST//RBFData_actor");
		statistics_file.append("RBFTEST//statistics");
		for (int i=0; i<dif; i++) {
			critic_file.append("0");
			actor_file.append("0");
			statistics_file.append("0");
		}

		char x [20];
		sprintf(x, "%u", epoch );
		critic_file.append(x);
		critic_file.append(".txt");
		actor_file.append(x);
		actor_file.append(".txt");
		statistics_file.append(x);
		statistics_file.append(".txt");


		ofstream RBF_TEST_critic;
		ofstream RBF_TEST_actor;
		ofstream RBF_TEST_statistics;


		if (write_critic)
			RBF_TEST_critic.open(critic_file.c_str());
		if (write_actor)
			RBF_TEST_actor.open(actor_file.c_str());

		if (write_critic || write_actor) {
			int min = -M_PI;
			int max = M_PI;
			double xin [2];
			for (xin[0]=min; xin[0]<max; xin[0]+=(M_PI/10.0)) {
				for (xin[1]=min; xin[1]<max; xin[1]+=(M_PI/10.0)) {
					if (write_critic){
						//critic
						double yout_critic;
						ngnet->incsbox_output(&VALUE, xin, &yout_critic, &nbasis);
						RBF_TEST_critic<<xin[0]<<", "<<xin[1]<<", "<<yout_critic<<"\n";
					}
					if (write_actor){
						//actor
						double yout_actor;
						//ngnet_actor->incsbox_output(&VALUE_ACTOR, xin, &yout_actor, &nbasis_value);
						RBF_TEST_actor<<xin[0]<<", "<<xin[1]<<", "<<yout_actor<<"\n";
					}
				}
				if (write_critic)  RBF_TEST_critic<<"\n";
				if (write_actor)   RBF_TEST_actor<<"\n";
			}
			if (write_critic) { RBF_TEST_critic<<"\n"; RBF_TEST_critic.close();}
			if (write_actor)  { RBF_TEST_actor<<"\n";RBF_TEST_actor.close();}
		}


		if (write_statistics) {
			RBF_TEST_statistics.open(statistics_file.c_str());
			//statistics
			RBF_TEST_statistics<<"//////////////////////////////////////////\n";
			RBF_TEST_statistics<<"Critic structure values:\n";
			RBF_TEST_statistics<<"------------------------\n";
			RBF_TEST_statistics<<"nbasis: "<<nbasis<<"\n";
			RBF_TEST_statistics<<"Error threshold: "<<error_thresh_critic<<"\n";
			RBF_TEST_statistics<<"Unit activation threshold: "<<unit_activation_thresh_critic<<"\n";
			RBF_TEST_statistics<<"//////////////////////////////////////////\n";
			RBF_TEST_statistics<<"actor structure values:\n";
			RBF_TEST_statistics<<"-----------------------\n";
			RBF_TEST_statistics<<"nbasis: "<<nbasis_value<<"\n";
			RBF_TEST_statistics<<"Error threshold: "<<error_thresh_actor<<"\n";
			RBF_TEST_statistics<<"Unit activation threshold: "<<unit_activation_thresh_actor<<"\n";
			RBF_TEST_statistics<<"//////////////////////////////////////////\n";
			RBF_TEST_statistics<<"Reinforcement learning values:\n";
			RBF_TEST_statistics<<"-------------------------------\n";
			RBF_TEST_statistics<<"critic learning rate:"<<rate_valuefunction<<"\n";
			RBF_TEST_statistics<<"actor learning rate:"<<rate_policy<<"\n";
			RBF_TEST_statistics.close();
		}


	}
}

double NimmOG::value_function(double *x /*in*/)
{
	double Value;

	ngnet->incsbox_output(&VALUE, x, &Value, &nbasis);

	return Value;
}
