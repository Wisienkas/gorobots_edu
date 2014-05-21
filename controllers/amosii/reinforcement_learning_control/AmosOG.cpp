/*************************************************************************
 * reinforcement learning controller for AMOS robot in
 * one goal scenario
 * Created/modified by Bassel Zeidan Dec 10, 2013
 **************************************************************************/


#include "AmosOG.h"
#include <selforg/controller_misc.h>
#include "utils/rbf-framework/ngnet.h"
#include "hexapod_neurocontroller.h"

/////////////////////////////////////////////////////////////////////////
//General information: AMOS - One goal scenario
/*************************************************************************
 * Note: 1) one RBF network (with two outputs) is used for the actor and the critic to speed up the execution.
 * 		 However, the learning rule is different for these two outputs.
 *		 2) In each learning iteration, the learning samples (TD error, inputs, outputs) for each time step
 *		 	are saved in a list called (samples) until the termination of the iteration and then the updates
 *		 	are done over the RBF network (learning in a batch mode)
 *		 	submit_sample() ---> function to save a learning sample (called at each time step)
 *		 	update_rbf_network() ---> perform updates over the RBF network (called after the learning iteration is terminated)
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
 * 		-1 when the robot hits an obstacle
 *
 * Parameters:
 * 		The actor's learning rate = 0.02
 * 		The critic's learning rate = 0.05
 * 		The discount factor = 0.999
 * 		exploration scale factor = 1.1
 *
 *
 * Actor:
 * 		Number of inputs: 3
 * 		inputs: the relative angle from one Transmitter, the relative distance from two transmitters
 * 		Number of hidden neurons: 512
 * 		Number of outputs: one control output. (the second output of the RBF network)
 *
 * Critic:
 * 		Number of inputs: 3
 * 		inputs: the relative angle from one Transmitter, the relative distance from two transmitters
 * 		Number of hidden neurons: 512
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
 * * * WRITE_AVERAGE_OF_EPOCHS (1) write the number of trials that are needed before convergence (to the average file)
 *   (0) Otherwise
 * * WRITE_INPUT_OUTPUT_SIGNALS (1) write input - output signals to the file for each time step (0) Otherwise
 *************************************************************************************/

#define WRITE_RBF_TO_FILE 0
#define USE_SAVED_RBF 1
#define WRITE_CRITIC_ACTOR_WEIGHTS_TO_TEXT_FILE 0
#define TERMINATE_AFTER_LEARNING 1
#define WRITE_INPUT_OUTPUT_SIGNALS 0
#define WRITE_AVERAGE_OF_EPOCHS 0


#define rbf_num_units 512
#define rbf_num_IN 3
#define rbf_num_out 2


// mask unit-wide variables
namespace {
  NGNet* ngnet;
  Cell VALUE(rbf_num_units, rbf_num_IN, rbf_num_out);
  HexapodNeuroMotionGenerator* motiongenerator;
  int current_epoch;
  int pause_step;
  bool written = false;
}

#ifndef clip
#define	clip(x/*input*/,l /*lower limit*/,u /*upper limit*/)( ((x)<(l)) ? (l) : ((x)>(u)) ? (u) : (x))
//if x < l , x = l and if x > u, x = u, else  x = x
#endif

#ifndef min
#define min(x/*e.g., 1*/, y) ((x <= y) ? x : y)
#endif

#ifndef max
#define max(x/*e.g., 0*/, y) ((x >= y) ? x : y)
#endif

using namespace matrix;
using namespace std;


AmosOG::AmosOG(const AmosOGConf& _conf)
: AbstractController("AmosOG", "$Id: "), conf(_conf)
{

	pause_step = 10;
	td_error=0.0;
	current_epoch= 0;
	motiongenerator = new HexapodNeuroMotionGenerator();
	ngnet = new NGNet(rbf_num_IN, rbf_num_out);

	preprogrammed_steering_control_active =false;
	t=0;
	//for manual steering
	mc[0]=0;
	mc[1]=0;
	mc[2]=0;
	mc[3]=0;
	resetLearning_RL = 0;
	output_actor = 0.0;
	input_angle_s.resize(4);
	ias_w.resize(2);
	etrace_io.resize(2);
	etrace_io_old.resize(2);

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
	addInspectableValue("xt[_ias3]",&xt[_ias3],"xt[_ias3]");
	addInspectableValue("input_distance_s",&input_distance_s,"input_distance_s");
	addInspectableValue("input_distance_s2",&input_distance_s2,"input_distance_s2");
	addInspectableValue("input_angle_s1", &input_angle_s.at(0),"input_angle_s1");
	addInspectableValue("input_angle_s2", &input_angle_s.at(1),"input_angle_s2");
	addInspectableValue("input_angle_s3", &input_angle_s.at(2),"input_angle_s3");
	addInspectableValue("input_angle_s4", &input_angle_s.at(3),"input_angle_s4");
	addInspectableValue("distance", &distance,"distance");

	initialized = false; //-----------------------------------------------(FRANK)
	use_Learned_weightsv2 = false;//--------------true-----------------------------------------------TEST
	large_input_rangev2 = false;
	terminate_now = false;
	no_learning_noise = true;

};
AmosOG::~AmosOG()
{
}
//controller initialization
void AmosOG::init(int sensornumber, int motornumber, RandGen* randGen)
{
	stepnumber_t = 0;
	++current_epoch;
	if(!initialized) //-----------------------------------------------(FRANK)
	{
		w_ico = 0.5;
		w_ac= 0.5;
		rate = 0.0005;
		ss_rate = 0.0005;
		learn_combined_weights = true; //true ;
		reduce_noise = 0;
		reduce_noise_ICO = 0;
		number_sensors = sensornumber;
		number_motors = motornumber;
		old_state=0;
		old_action=0;
		vmax = 0.05;
		vmin = -0.05;
		deri_alpha = 0.0;
		alpha_old = 0.0;
		alpha_old2 = 0.0;
		alpha_old3 = 0.0;
		alpha_old4 = 0.0;
		k_ico[0] = 0.0;
		k_ico[1] = 0.0;
		k_ico[2] = 0.0;
		k_ico[3] = 0.0;

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
		nbasis_value =0;
		xdim = XDIM /*=2 inputs, Set for different system */;
		udim = 1;/*output of critic always = 1, Set for different system */;
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

		hit_times = 0;

		// initial weight of actors
		for(int i = 0; i< WDIM/*8*/; i++)
		{
			k[i] = 0.0;
		}
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
		rate_policy = 0.02; //0.001;//0.01;//0.01;// learning rate of actor
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
	}

}
//calculate relative angles
void AmosOG::calculateAnglePositionFromSensors(const sensor* x_) //as it says on the tin
{
	//78 79 115 116
	//alpha_old =  alpha;
	double alpha_var;
	double alpha_var_temp;

	for (int i=0; i<2; i++) {
		if (sign(x_[62+(i*10)])>0)
		{
			alpha_var = atan(x_[63+(i*10)]/x_[62+(i*10)]) * 180 / M_PI;
		} else {
			alpha_var_temp = -1*atan (x_[63+(i*10)]/x_[62+(i*10)]) * 180 / M_PI;
			if (alpha_var_temp<=0)
				alpha_var = (-90 + (-90-alpha_var_temp));
			else alpha_var = ( 90 + ( 90-alpha_var_temp));
		}
		input_angle_s.at(i) =  alpha_var/180.*M_PI;
	}

	if (sign(x_[78])>0) {
		alpha_var = atan(x_[79]/x_[78]) * 180 / M_PI;
	} else {
		alpha_var_temp = -1*atan (x_[79]/x_[78]) * 180 / M_PI;
		if (alpha_var_temp<=0)
			alpha_var = (-90 + (-90-alpha_var_temp));
		else alpha_var = ( 90 + ( 90-alpha_var_temp));
	}
	input_angle_s.at(2) =  alpha_var/180.*M_PI;


	if (sign(x_[81])>0) {
		alpha_var = atan(x_[82]/x_[81]) * 180 / M_PI;
	} else {
		alpha_var_temp = -1*atan (x_[82]/x_[81]) * 180 / M_PI;
		if (alpha_var_temp<=0)
			alpha_var = (-90 + (-90-alpha_var_temp));
		else alpha_var = ( 90 + ( 90-alpha_var_temp));
	}
	input_angle_s.at(3) =  alpha_var/180.*M_PI;
}
//calculate distances
void AmosOG::calculateDistanceToGoals(const sensor* x_)
{
	double scale = 0.1;
	max_dis = position_x_robot*position_x_robot; //normalize
	distance = (sqrt(pow(x_[63/*11 y*/],2)+pow(x_[62/*10 x*/],2)));
	//distance *= scale;
	input_distance_s = distance;///100;//85;//max_dis;
	//input_distance_s -= 2;

	max_dis2 = position_x_robot*position_x_robot; //normalize
	distance2 = (sqrt(pow(x_[73/*y*/],2)+pow(x_[72/*x*/],2)));
	input_distance_s2 = distance2;///100;//85;//max_dis;
	distance2 *= scale;

	//max_dis2 = position_x_robot*position_x_robot; //normalize
	distance3 = (sqrt(pow(x_[79/*y*/],2)+pow(x_[78/*x*/],2)));
	//input_distance_s2 = distance2;///100;//85;//max_dis;
	distance3 *= scale;
}
//low pass filters
void AmosOG::inputSensorSignals(const sensor* x_)
{

		double gain_r = 0.992;//0.99;
		for (int i=0; i<4; i++) {
			low_passes_old[i] = low_passes[i];
			if (stepnumber_t != 0)
				low_passes[i] = ((1-gain_r)*input_angle_s.at(i))+(low_passes_old[i]*gain_r);
			else low_passes[i] = input_angle_s.at(i);
		}
	///////////////////////////////////////////////////////////
	/////////////distances

	double gain_ir = 0.95;

	lowpass_dis_x_old = lowpass_dis_x;
	lowpass_dis_y_old = lowpass_dis_y;
	lowpass_dis_z_old = lowpass_dis_z;

	lowpass_dis_x = (1-gain_ir)*((distance))+lowpass_dis_x_old*gain_ir;
	lowpass_dis_y = (1-gain_ir)*((distance2))+lowpass_dis_y_old*gain_ir;
	lowpass_dis_z = (1-gain_ir)*((distance3))+lowpass_dis_z_old*gain_ir;

}
//low pass filters for IR signals
void AmosOG::get_ir_signals(const sensor* x_)
{
	//Calculating IR sensors

	ir_l = x_[25];
	ir_r = x_[26];

	input_ir_s = ir_l-ir_r;

	//preprocessing using a low pass filter
	double gain_ir = 0.99;

	irl_lowpass_old = irl_lowpass;
	irr_lowpass_old = irr_lowpass;

	irl_lowpass = (1-gain_ir)*ir_l+irl_lowpass_old*gain_ir;
	irr_lowpass = (1-gain_ir)*ir_r+irr_lowpass_old*gain_ir;
}
//Print learning parameters (NOT USED HERE)
void AmosOG::print_to_cout(int steps_period)
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
				//std::cin>>i;
			}
	}
}
//reset function (called when a learning iteration is terminated)
void AmosOG::reset_params()
{
	xt_ico_lowpass = 0;
	xt_ico_lowpass2= 0;
	irl_lowpass = 0;
	irr_lowpass = 0;
	lowpass_dis_x = 0;
	lowpass_dis_y = 0;
	//exploration_old = 0;


	stepnumber_t = 0;
	++current_epoch;
	if (hit_the_goal && (current_epoch>200))
		reduce_with_time_counter++;

	exploration_lowpass_g[0] = gauss()/10.0;

	for(int i =0; i< UDIM /*2*/; i++) {
		exploration_lowpass_g[i] = 0;
	}

	for (int i=0; i< XDIM; i++)
		input_old[i] = 0;


	for (int i=0; i<this->samples.size(); i++) {
		update_critic_actor( samples.at(i).inputs.data(),
						samples.at(i).tds.at(0),
						rate_valuefunction,
						samples.at(i).p_error,
						rate_policy,
						samples.at(i).tds.at(1)
							);

	}

	samples.clear();
	std::cout<<"epoch number"<<ntrial<<std::endl;


	if (WRITE_CRITIC_ACTOR_WEIGHTS_TO_TEXT_FILE)
			VALUE.print_weights("Weights", rbf_num_units, rbf_num_out);

	//ngnet->reset_network(&VALUE_ACTOR, &nbasis_value);
	//ngnet_actor->reset_network(&VALUE, &nbasis);

	//ngnet->reset_incsbox(&VALUE);
	//ngnet_actor->reset_incsbox(&VALUE_ACTOR);
	//terminate_now = true;
}
//save input signals to a file
void AmosOG::save_inputs()
{
	ofstream _file;
	_file.open("inputs_log", std::fstream::app);
	_file<<xt[0]<<" "<<xt[_ias1]<<" "<<xt[_ias2]<<"\n";
	_file.close();

}
//save output signal to a file
void AmosOG::save_output()
{
	ofstream _file;
	_file.open("output_log", std::fstream::app);
	_file<<actual_out<<"\n";
	_file.close();
}
//save input signals for the next time step
void AmosOG::copy_input(double* inputs, double* inputs_old)
{
	for (int i=0; i<XDIM; i++) {
		inputs_old[i] = inputs[i];
	}
}
//controller step function (called at every time step)
void AmosOG::inner_controller(const sensor* x_, int number_sensors, motor* y_, int number_motors)
{
	double scale = 0.5;// 1.0;
	static int iter = 0; // Initial only one time
	static int iter_old = 0;
	static int trial_n = 0;
	static double Vt_old = 0.0;// Initial only one time
	static double rt_old = 0.0;
	double  rt_reducenoise;
	//----AC network parameters------------//

	if (WRITE_INPUT_OUTPUT_SIGNALS) {
		save_inputs();
		save_output();
	}

	iter_old = iter;

	//ADD
	rt_reducenoise = reward_function(xt, ut);
	if(rt_reducenoise < 0)
		reduce_noise -= 5;
	else if(rt_reducenoise > 0.0) {
			reduce_noise += 3;
		}

	if(reduce_noise<0)
		reduce_noise = 0;
	//ADD

	exp_output_decay = exp(-reduce_noise/200.0);//100);

	//std::cout<<"iterations = "<<stepnumber_t<<std::endl;



	/*---(3) Cal reward-----------------------------*/
	rt = reward_function(xt, ut);

	/*---(1) Output from current policy--------------*/
	double out [2];
	get_RBF_network_out(xt, out);
	Vt = out[0];
	ut[0] = out[1];

	/*** LEARNING STATE*******************/
	//if(ntrial < MAX_TRIAL)// && distance>d_threshold)
	{

		td_error = rt_old + (gam*Vt) - Vt_old;

		double pure_error = rt_old + Vt - Vt_old;

		submit_sample(input_old, td_error, pure_error, exploration_old);

		 if(resetLearning_RL == 1)
			 checkreset = 10;
		 else
			 checkreset = 0;
		 iter++;
		 resetLearning_RL++;
	}

	////save for next iteration
	copy_input(xt, input_old);
	exploration_old = actual_out - actor_out;
	Vt_old = Vt;
	rt_old = rt;


	failure_flag = check_limit(xt);
}
//step function (called at every time step)
void AmosOG::step(const sensor* x_, int number_sensors, motor* y_, int number_motors)
{

	int conntroller_step = 1;
	set_control_wave = true;

	/*****AFTER Fail->RESET*******************/
	if(resetLearning_RL == 0 /*when approach goal or hit obstacles*/)
	{
		failure_flag = 0;
		reset_params();


		//Vt_old = value_function(xt/*inputs*/); //Replaced by ESN // RESET p_old or reset Vt_old = V1 // as expected Max
		//Vt = value_function(xt/*inputs*/); //Replaced by ESN // RESET p_old or reset Vt_old = V1 // as expected Max
		//rt_old = 0;
		//////////////////////////////////////////////////////////
		ntrial++;
		//iter = 0;
		acum_reward = 0.0; // RESET acc reward // may need to analyze or evaluate system
		sig_out_log[ntrial] = sig_out[0]; // may need to analyze or evaluate system
		resetLearning_RL++;
	}



	/////print some stuff
	//print_to_cout(1000);
	///////////////////////////////////////////////////

	calculateAnglePositionFromSensors(x_);

	//---3) Calculating distance to the goals 1-4

	calculateDistanceToGoals(x_);

	//get ir signals
	get_ir_signals(x_);

	irl_back = x_[8];
	irr_back = x_[9];

	inputSensorSignals(x_);

	double* dummy;

	//inputs
	xt[_ias0] = low_passes[3];
	xt[_ias1] = lowpass_dis_y;
	xt[_ias2] = lowpass_dis_z;

	//if (((stepnumber_t % conntroller_step == 0) || (reward_function(xt, dummy) != 0)) && (!failure_flag)) {
	inner_controller(x_, number_sensors, y_, number_motors);
	//}
	motiongenerator->motor_command_interpreter(ut[0], y_);
	//motiongenerator->motor_command_interpreter(0, y_);



	stepnumber_t++;
}
#include <iostream>
#include <string.h>
void AmosOG::stepNoLearning(const sensor* x_, int number_sensors,
		motor* y_, int number_motors)
{

}
double AmosOG::sigmoid(double num)
{
	return 1./(1.+exp(-num));
}
double AmosOG::tanh(double num)
{
	return 2./(1.+exp(-2*num))-1.;
}
/*************************************************************
 *	Gaussian random variable: just a sum of uniform distribution
 *	Average = 0, Variance = 1, (1.2)
 *************************************************************/
double AmosOG::gauss()
{
	double	sum;
	int 	i;

	for( sum = i = 0; i < 12; i++)
		sum += (1.0*(double)rand()/(RAND_MAX));
	return(sum - 6.0);
}
//reward function
double AmosOG::reward_function(double *x /*state*/, double *u /*action*/)
{

	double r_total = 0.0;

	if(distance < 1.1) {
		return 1;
	}


	if ((irl_lowpass > 0.07) || (irr_lowpass > 0.07))
		return -1;

	return r_total;
}
//check the termination of the iteration (using Infrared signals)
int AmosOG::check_limit(double *x)//, double irr_back, double irl_back)
{

	int flag=0;

	if (termination_counter > 10) {
		if (!written) {
			written = true;
			if (WRITE_AVERAGE_OF_EPOCHS) {
				std::ofstream myfile;
				myfile.open ("Results_amos.txt", std::ios::app);
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

	if (current_epoch > 1500)
	{
		if (!written) {
			written = true;
			if (WRITE_AVERAGE_OF_EPOCHS) {
				std::ofstream myfile;
				myfile.open ("Results_amos.txt", std::ios::app);
				myfile<<"Fail to learn with 1500 iteration"<<std::endl;
				myfile.close();
			}
			flag = 1;
			terminate_now = true;
		}
		terminate_now = true;
	}




	if(irl_lowpass > 0.3 || irr_lowpass > 0.3) {
		flag = 1;
	}

	if (stepnumber_t > 3400)
		flag = 1;



	return flag;


}
//Initialization of the RBF networks
void AmosOG::init_funcapprox()
{
	int i;
	int state_index[XDIM /*input, e.g., 4*/];
	double xc[XDIM /*input, e.g.,4*/];

	ngnet->init_incsbox(&VALUE,XDIM , 2 /*1 one output*/);
	ngnet->reset_incsbox(&VALUE);

	//***Set it differently according to a number of your inputs, here only 2 inputs
	basis[_ias0] = 8; // N_IN0_RBF/*3*/; // Number of RBF
	basis[_ias1] = 8; // N_IN1_RBF/*3*/; // Number of RBF
	basis[_ias2] = 8;


	//***Set it differently according to a number of your inputs, here only 3 inputs
	//1) Input limitation MAX MIN
	xmax[_ias0] = MAX_IN0;
	xmin[_ias0] = MIN_IN0;

	xmax[_ias1] = 1;
	xmin[_ias1] = 0;

	xmax[_ias2] = 1;
	xmin[_ias2] = 0;

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
	{
		for(i=0;i<xdim /*2 inputs*/;i++)
			xc[i]  = (xmin[i]+basis_width[i])+state_index[i]*2.0*basis_width[i];
		ngnet->put_incsbox(&VALUE, XDIM , 2, xc, wbasis/*1/Variance*/, &nbasis);
	}
}
//the exploration function
double AmosOG::get_exploration(double Vt) {

	double y;
	double V1, V0;
	double max_value, min_value;
	//double amplified_value;
	double scale;
	double gain = 0.999;

	//double gain =  0.9997;//0.99;//0.6;

	if (Vt > vmax)
		vmax = Vt;
	if (Vt < vmin)
		vmin = Vt;


	double low_pass_value = ((1-gain)*gauss()) + ((gain)*(exploration_old));
	//save old value


	V1 = vmax / 2.0;//0.1;
	V0 = vmin / 2.0;//-0.1;


	y = (V1-Vt)/(V1-V0);
	max_value = max(0, y);
	min_value = min(/*0.1*/1 ,max_value); //changing from 1 to 0.1



	//std::cout<<"min_value"<<min_value<<std::endl;

	scale = 1.1*min_value;//perfect 0.08;
	//amplified_value = scale*min_value;

	exploration_old = low_pass_value;//*min_value;


	return scale*low_pass_value;
}
//get the next action and the next V value
void AmosOG::get_RBF_network_out(double* in, double* out) {

	double Value [2];
	ngnet->incsbox_output(&VALUE, in, Value, &nbasis);
	out[0] = Value[0];

	if (!USE_SAVED_RBF) {

		double exp = get_exploration(Value[0])*exp_output_decay;
		exp = clip(exp, -0.3, 0.3);
		Value[1] = clip(Value[1], -0.3, 0.3);


		if (exp_output_decay > 0.001) {
			termination_counter = 0;

			if (current_epoch % 2 == 0)
				out[1] = exp;
			else
				out[1] = Value[1] + exp;
		} else {

			if (last_epoch != current_epoch) {
				last_epoch = current_epoch;

				termination_counter++;
			}

			out[1] = Value[1];
		}

		exploration_out = exp;
		actor_out = Value[1];
		actual_out = out[1];
	} else {
		exploration_out = 0;
		actor_out = Value[1];
		actual_out = actor_out;
		out[1] = actual_out;
	}
}
//update RBF network (the same network is used for the critic and the actor)
void AmosOG::update_critic_actor(double* input_old, double td_error
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
//save the current learning sample (taken from the current time step values)
//these sample are used after the termination of the current iteration to update RBF network
void AmosOG::submit_sample(double* input_old, double td_error, double pure_error, double actor_update_value) {
	Sample s;
	for (int i=0; i<3; i++)
		s.inputs.push_back(input_old[i]);
	s.tds.push_back(td_error);
	s.tds.push_back(actor_update_value);
	s.p_error = pure_error;
	samples.push_back(s);
}
