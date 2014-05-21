/*************************************************************************
 * reinforcement learning controller (with ICO) for AMOS robot in
 * multiple goal scenario.
 * Created/modified by Bassel Zeidan Dec 10, 2013
 **************************************************************************/
//General information: AMOS - multiple goal scenario
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
 * 		-1 when the robot hits an obstacle
 *
 * Parameters:
 * 		The actor's learning rate = 0.03
 * 		The critic's learning rate = 0.05
 * 		The discount factor = 0.9999
 * 		exploration scale factor = 0.4
 * 		ICO learning rate = 0.6
 *
 *
 * Actor:
 * 		Number of inputs: 4
 * 		inputs: the relative angle from one Transmitter (Green), the relative distance from two transmitters (2 white transmitters)
 * 		the fourth input is the SD input (Subgoal definer)
 * 		Number of hidden neurons: 1512
 * 		Number of outputs: one control output. (the second output of the RBF network)
 *
 * Critic:
 * 		Number of inputs: 4
 * 		inputs: the relative angle from one Transmitter (Green), the relative distance from two transmitters (2 white transmitters)
 * 		the fourth input is the SD input (Subgoal definer)
 * 		Number of hidden neurons: 1512
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
 * * WRITE_INPUTS_OUTPUT_SIGNALS: (1) saves inputs, outputs signals
 * * In RobotState.cpp file there is one parameter (WRITE_AVERAGE) which determine if
 *   the number of trials that were needed until the robot's policy converge to the optimal
 *   one should be recorded. (1) the number is recorded (0) otherwise
 *************************************************************************************/

#define WRITE_RBF_TO_FILE 0
#define USE_SAVED_RBF 1
#define WRITE_CRITIC_ACTOR_WEIGHTS_TO_TEXT_FILE 0
#define TERMINATE_AFTER_LEARNING 0
#define WRITE_INPUTS_OUTPUT_SIGNALS 0
#define WRITE_ICO_Weight 0



#include "AmosMG.h"
#include <selforg/controller_misc.h>
#include "utils/rbf-framework/ngnet.h"
#include "hexapod_neurocontroller.h"

#define rbf_num_units 1512
#define rbf_num_IN 4
#define rbf_num_out 2

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


#ifndef absolute
#define absolute(x) (((x) < 0) ? -(x) : (x))
#endif

using namespace matrix;
using namespace std;

#define max_action 0.8

namespace {
  NGNet* ngnet;
  Cell VALUE(rbf_num_units, rbf_num_IN, rbf_num_out);
  HexapodNeuroMotionGenerator* motiongenerator;

  //----AC network parameters------------//
  int current_epoch;

  int pause_step;
}



AmosMG::AmosMG(const AmosMgConf& _conf)
: AbstractController("AmosMG", "$Id: "), conf(_conf)
{

	pause_step = 10;
	ngnet = new NGNet(rbf_num_IN, rbf_num_out);

	td_error=0.0;
	current_epoch= 0;
	motiongenerator = new HexapodNeuroMotionGenerator();

	preprogrammed_steering_control_active =false;
	t=0;
	//for manual steering
	mc[0]=0;
	mc[1]=0;
	mc[2]=0;
	mc[3]=0;
	resetLearning_RL = 0;
	output_actor = 0.0;

	//-----Set vector size------------------//
	input_angle_s.resize(NUMBER_OF_BALLS);
	ias_w.resize(2);
	etrace_io.resize(2);
	etrace_io_old.resize(2);


	//-----Plot values----------------------//

	addInspectableValue("VT",&real_v,"Vt");
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
	addInspectableValue("distance", &distances[0],"distance");
	addInspectableValue("ico_out", &ico_out,"ico_out");
	addInspectableValue("ico_weight", &ico_weight,"ico_weight");
	addInspectableValue("temp_nearby_distance", &temp_nearby_distance,"temp_nearby_distance");
	addInspectableValue("prediction_observer", &prediction_observer,"prediction_observer");
	addInspectableValue("reflex_observer", &reflex_observer,"reflex_observer");

	//----Protecting Reset parameters------//
	initialized = false; //-----------------------------------------------(FRANK)
	use_Learned_weightsv2 = false;//--------------true-----------------------------------------------TEST
	large_input_rangev2 = false;
	terminate_now = false;
	//No learning noise
	no_learning_noise = true;
	//----Protecting Reset parameters------//

};
AmosMG::~AmosMG() {
}
void AmosMG::init(int sensornumber, int motornumber, RandGen* randGen)
{
	stepnumber_t = 0;
	++current_epoch;
	if(!initialized) //-----------------------------------------------(FRANK)
	{
		Vt_old = 0;
		rt_old = 0;
		w_ico = 0.5;
		w_ac= 0.5;
		rate = 0.0005;
		ss_rate = 0.0005;
		reduce_noise = 0;
		reduce_noise_ICO = 0;
		terminate_immediatly = false;

		number_sensors = sensornumber;
		number_motors = motornumber;

		old_state=0;
		old_action=0;

		deri_alpha = 0.0;
		alpha_old = 0.0;
		alpha_old2 = 0.0;
		alpha_old3 = 0.0;
		alpha_old4 = 0.0;

		//---ICO network parameters------------//
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
		rate_policy = 0.03; //0.001;//0.01;//0.01;// learning rate of actor
		rate_valuefunction = 0.05;// learning rate_critic of weights from hidden RBF neurons to its output neuron (critic)



		//--CRITIC
		td_limit = 1.0; // limit of clipping TDerror (critic)
		gam = 0.9999;//0.99999999999;//0.98;//0.98; // discount factor of Vt (critic)
		error_thresh_critic = 0.2; // threshold for critic weight changes
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

	for (int i=0; i<(NUMBER_OF_BALLS*3); i++) {
		old_x.push_back(0);
		x_lowpass.push_back(0);
	}
}
//low pass filter
void AmosMG::signal_filter(const sensor* x_) {
	double gain = 0.9;//0.93;//0.99;
	for (int i=0; i<(NUMBER_OF_BALLS*3); i++) {

		old_x[i] = x_lowpass[i];
		x_lowpass[i] = (gain*old_x[i]) + ((1-gain)*x_[82+i]);
	}
}
//calculate relative angles
void AmosMG::calculateAnglePositionFromSensors(const sensor* x_) //as it says on the tin
{
	int i=0;
	for (int counter=0; counter<NUMBER_OF_BALLS; ++counter)
	{
		//std::cout<<"iam here="<<x_[12 + i]<<", "<<x_[13 + i]<<std::endl;
		double alpha_value = 0;
		if (sign(x_lowpass[2 + i])>0)
			alpha_value = atan(x_lowpass[1 + i]/x_lowpass[2 + i]) * 180 / M_PI; // angle in degrees
		else {
			double alpha_value_tmp = -1*atan (x_lowpass[1 + i]/x_lowpass[2 + i]) * 180 / M_PI; // angle in degrees
			if (alpha_value_tmp<=0)
				alpha_value = (-90 + (-90-alpha_value_tmp));
			else alpha_value = ( 90 + ( 90-alpha_value_tmp));
		}
		input_angle_s.at(counter) = alpha_value/180.*M_PI;
		i+=3;
	}
}
//calculate distances
void AmosMG::calculateDistanceToGoals(const sensor* x_)
{
	double distance_scale = 0.1;//1/100;

	for (int i=0; i<NUMBER_OF_BALLS; i++){
		max_dis = position_x_robot*position_x_robot; //normalize
		distance = (sqrt(pow(x_lowpass[1+(i*3) /*11 y*/],2)+pow(x_lowpass[2+(i*3)/*10 x*/],2)));
		distances[i] = distance*distance_scale;//85;//max_dis;
	}
}
//low pass filters for input signals
void AmosMG::inputSensorSignals(const sensor* x_)
{

	double gain_r = 0.9;//0.99;
	double gain_r2 = 0.98;//0.995;

	for (int i=0; i<NUMBER_OF_BALLS; i++) {
		low_pass_angles_old[i] = low_pass_angles[i];
		low_pass_angles[i] = ((1-gain_r2)*input_angle_s.at(i)) + (low_pass_angles_old[i]*gain_r2);//red

		low_pass_dis_old[i] = low_pass_dis[i];
		low_pass_dis[i] = ((1-gain_r)*distances[i]) + (low_pass_dis_old[i]*gain_r);//red
	}
}
//low pass filter for IR signals
void AmosMG::get_ir_signals(const sensor* x_)
{
	ir_l = x_[25];
	ir_r = x_[26];

	input_ir_s = ir_l-ir_r;

	//preprocessing using a low pass filter
	double gain_ir = 0.95;

	irl_lowpass_old = irl_lowpass;
	irr_lowpass_old = irr_lowpass;

	irl_lowpass = (1-gain_ir)*ir_l+irl_lowpass_old*gain_ir;
	irr_lowpass = (1-gain_ir)*ir_r+irr_lowpass_old*gain_ir;
}
//printing function (NOT USED)
void AmosMG::print_to_cout(int steps_period)
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

bool written = false;
bool done = false;

bool locker = true;
//reset function (called after each trial)
void AmosMG::reset_params()
{
	std::cout<<"enter reset"<<std::endl;

	locker = true;

	if (WRITE_INPUTS_OUTPUT_SIGNALS) {
		done = true;
		terminate_immediatly = true;
	}

	if (WRITE_ICO_Weight) {
		ofstream _file;
		_file.open("ICO_weight", std::fstream::app);
		_file<<ico_weight<<"\n";
		_file.close();
	}

	xt_ico_lowpass = 0;
	xt_ico_lowpass2= 0;
	irl_lowpass = 0;
	irr_lowpass = 0;
	lowpass_dis_x = 0;
	lowpass_dis_y = 0;


	Vt_old = 0;
	rt_old = 0;


	//exploration_old = 0;


	stepnumber_t = 0;
	++current_epoch;
	//if (hit_the_goal && (current_epoch>200))
	//	reduce_with_time_counter++;

	exploration_lowpass_g[0] = gauss()/10.0;

	for(int i =0; i< UDIM /*2*/; i++) {
		exploration_lowpass_g[i] = 0;
	}

	for (int i=0; i< XDIM; i++)
		input_old[i] = 0;


	if (!USE_SAVED_RBF) {

		if (ntrial != 1) {
		for (int i=1; i<this->samples.size(); i++) {
			update_critic_actor( samples.at(i).inputs.data(),
							samples.at(i).tds.at(0),
							rate_valuefunction,
							samples.at(i).p_error,
							rate_policy,
							samples.at(i).tds.at(1)
								);
		}

		}
	}

	/*for (int i=0; i<samples.size(); i++){
		std::cout<<"TD["<<i<<"]="<<samples[i].tds[0]<<"\n"
				 <<"inputs="<<samples[i].inputs[0]<<","<<samples[i].inputs[1]<<","<<samples[i].inputs[2]<<"\n"
				 <<std::endl;
	}*/

	samples.clear();
	std::cout<<"epoch number"<<ntrial<<std::endl;



	//terminate_immediatly =
			//robotstate.reset_robot_state();
	//std::cout<<terminate_immediatly<<std::endl;
	//ngnet->reset_network(&VALUE_ACTOR, &nbasis_value);
	//ngnet_actor->reset_network(&VALUE, &nbasis);

	//ngnet->reset_incsbox(&VALUE);
	//ngnet_actor->reset_incsbox(&VALUE_ACTOR);
	for (int i=0; i<NUMBER_OF_BALLS; i++) {
		low_pass_angles_old[i] = 0;
		low_pass_dis_old[i] = 0;
	}

	for (int i=0; i<(NUMBER_OF_BALLS*3); i++) {
		old_x[i] = 0;
		//x_lowpass[i] = 0;
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



	if (WRITE_CRITIC_ACTOR_WEIGHTS_TO_TEXT_FILE)
		VALUE.print_weights("Weights", rbf_num_units, rbf_num_out);
	std::cout<<"out reset"<<std::endl;
}
//save current inputs for next step
void AmosMG::copy_input(double* inputs, double* inputs_old)
{
	for (int i=0; i<XDIM; i++) {
		inputs_old[i] = inputs[i];
	}
}
//save input signals' values to a file
void AmosMG::save_inputs()
{
	ofstream _file;
	_file.open("inputs_log", std::fstream::app);
	_file<<xt[0]<<" "<<xt[_ias1]<<" "<<xt[_ias2]<<" "<<xt[3]<<"\n";
	_file.close();
}
//save output signals' values to a file
void AmosMG::save_output()
{
	ofstream _file;
	_file.open("output_log", std::fstream::app);
	_file<<actual_out<<"\n";
	_file.close();
}
//Controller step function (called for each time step)
void AmosMG::inner_controller(const sensor* x_, int number_sensors, motor* y_, int number_motors)
{
	double scale = 0.5;// 1.0;
	static int iter = 0; // Initial only one time
	static int iter_old = 0;
	static int trial_n = 0;
	//static double Vt_old = 0.0;// Initial only one time
	//static double rt_old = 0.0;
	double  rt_reducenoise;
	//----AC network parameters------------//

	failure_flag = check_limit(xt);

	iter_old = iter;
	signal_filter(x_);
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
	xt[0] = low_pass_dis[4];//distances[8];
	xt[1] = low_pass_dis[5];//distances[9];
	xt[2] = low_pass_angles[6];//low_pass_angles[6];
	xt[3] = robotstate.get_state_array();

	//std::cout<<"Current V value: "<<Vt<<std::endl;
	if (WRITE_INPUTS_OUTPUT_SIGNALS) {
		//if (!done) {
			save_inputs();
			save_output();
		//}
	}



	//ADD

	//exp_output_decay = exp(-reduce_noise/200.0);//100);

	//std::cout<<"iterations = "<<stepnumber_t<<std::endl;

	int number = 0;
	//if (distances[0] < 0.12)
		//std::cin>>number;
	//std::cout<<distances[0]<<std::endl;


	/*---(3) Cal reward-----------------------------*/
	rt = reward_function(xt, ut);


	robotstate.submit_step_number(stepnumber_t);

	/*bool checker = false;
	for (int i=0;i <5; i++)
		if (distances[i] < 0.07)
			checker = true;


	if (!checker) locker = false;

	if (!locker) {*/

	if (stepnumber_t > 40) {
	//////////////////////////////ICO
	bool ICO_ACTIVE = true;
	//double ico_out;
	double input_scale = 1;//0.1;
	if ((ICO_ACTIVE) && (!failure_flag)) {
		int signal_index =robotstate.get_nearest_signal_index(distances);
		//std::cout<<"signal index"<<signal_index<<std::endl;
		if (signal_index != -1) {


			prediction_observer = 1;
			if (robotstate.Is_reflex_active(distances, signal_index))
				reflex_observer = 1;

			temp_nearby_distance = distances[signal_index];
			old_ico_value = ico_input;
			ico_input = absolute(low_pass_angles[signal_index]*input_scale);
			double ico_current_out = robotstate.get_ICO_out(ico_input, distances, signal_index);
			//get output
			if (low_pass_angles[signal_index] > 0)
				ico_out = - ico_current_out;
			else ico_out =  ico_current_out;
			ico_weight = robotstate.learn_ico(ico_input, absolute(old_ico_value), distances, signal_index)[0];
		} else {
			ico_out = 0;//robotstate.get_ICO_out(0);
			temp_nearby_distance = 0;

			prediction_observer = 0;
			reflex_observer = 0;
		}
	}
	///////////////////////////////////
	}


	/*---(1) Output from current policy--------------*/
	double out [2];
	get_RBF_network_out(xt, out);
	Vt = out[0];
	ut[0] = out[1];


	real_v = Vt / 100.0;

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



}
//step function (called for each time step)
void AmosMG::step(const sensor* x_, int number_sensors, motor* y_, int number_motors)
{

	int conntroller_step = 1;
	set_control_wave = true;

	/*****AFTER Fail->RESET*******************/
	if(resetLearning_RL == 0 /*when approach goal or hit obstacles*/)
	{
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
		failure_flag = 0;
	}




	//&& (stepnumber_t > 10)) {


	if ((failure_flag != 1) && (stepnumber_t > 0)) {

		/////print some stuff
		//print_to_cout(1000);
		///////////////////////////////////////////////////





		//if (((stepnumber_t % conntroller_step == 0) || (reward_function(xt, dummy) != 0)) && (!failure_flag)) {
		//if ((stepnumber_t % conntroller_step == 0) || (reward_function(xt, dummy) != 0))
		//if ((stepnumber_t % 5 == 0)) {
			inner_controller(x_, number_sensors, y_, number_motors);
			//std::cout<<"me again\n";
		//}
		//}
	}

//	if (stepnumber_t % 5 == 0)
		motiongenerator->motor_command_interpreter(ut[0], y_);
	//motiongenerator->motor_command_interpreter(0, y_);
	stepnumber_t++;
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

void AmosMG::stepNoLearning(const sensor* x_, int number_sensors,
		motor* y_, int number_motors)
{

}
double AmosMG::sigmoid(double num)
{
	return 1./(1.+exp(-num));
}

double AmosMG::tanh(double num)
{
	return 2./(1.+exp(-2*num))-1.;
}
/*************************************************************
 *	Gaussian random variable: just a sum of uniform distribution
 *	Average = 0, Variance = 1, (1.2)
 *************************************************************/
double AmosMG::gauss()
{
	double	sum;
	int 	i;

	for( sum = i = 0; i < 12; i++)
		sum += (1.0*(double)rand()/(RAND_MAX));
	return(sum - 6.0);
}
//reward function
double AmosMG::reward_function(double *x /*state*/, double *u /*action*/)
{
	if ((irl_lowpass > 0.04) || (irr_lowpass > 0.04))
		return -1;

	return robotstate.get_reward(distances);
}
//check the termination of the current iteration
int AmosMG::check_limit(double *x)//, double irr_back, double irl_back)
{

	int flag=0;

	/*if (termination_counter > 15) {
		if (!written) {
			written = true;
			std::ofstream myfile;
			myfile.open ("Results_amos.txt", std::ios::app);
			myfile<<"Trials number = "<<current_epoch<<std::endl;
			myfile.close();
			flag = 1;
			terminate_now = true;
		}
	}

	if (current_epoch > 1500)
	{
		if (!written) {
			written = true;
			std::ofstream myfile;
			myfile.open ("Results_amos.txt", std::ios::app);
			myfile<<"Fail to learn with 1500 iteration"<<std::endl;
			myfile.close();
			flag = 1;
			terminate_now = true;
		}
		terminate_now = true;
	}*/




	if(irl_lowpass > 0.4 || irr_lowpass > 0.4) {
		flag = 1;
	}

	/*if (stepnumber_t > 3400)
		flag = 1;*/

	if (robotstate.Final_goal_is_reached() )
	{
//std::cout<<"iaa mmmmmmmmmmmmmmmmmmmmmm 1";
			flag = 1;
	}
	if (robotstate.Is_end_of_phase()) {
//		std::cout<<"iaa mmmmmmmmmmmmmmmmmmmmmm 2";
		flag = 1;
	}

	return flag;


}
//	Initial RBF network
void AmosMG::init_funcapprox()
{
	int i;
	int state_index[XDIM /*input, e.g., 4*/];
	double xc[XDIM /*input, e.g.,4*/];

	ngnet->init_incsbox(&VALUE,XDIM , 2 /*1 one output*/);
	ngnet->reset_incsbox(&VALUE);

	//***Set it differently according to a number of your inputs, here only 2 inputs
	basis[_ias0] = 6;//8; // N_IN0_RBF/*3*/; // Number of RBF
	basis[_ias1] = 6;//8; // N_IN1_RBF/*3*/; // Number of RBF
	basis[_ias2] = 6;//8;
	basis[_ias3] = 7;//5;


	//***Set it differently according to a number of your inputs, here only 3 inputs
	//1) Input limitation MAX MIN
	xmax[_ias0] = 1.5;
	xmin[_ias0] = 0;

	xmax[_ias1] = 1.5;
	xmin[_ias1] = 0;

	xmax[_ias2] = MAX_IN2;
	xmin[_ias2] = MIN_IN2;

	xmax[_ias3] = 5;
	xmin[_ias3] = 0;

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
	for(state_index[_ias3]=0; state_index[_ias3]<basis[_ias3] /*3*/; state_index[_ias3]++) // angle right
	{
		for(i=0;i<xdim /*2 inputs*/;i++)
			xc[i]  = (xmin[i]+basis_width[i])+state_index[i]*2.0*basis_width[i];
		ngnet->put_incsbox(&VALUE, XDIM , 2, xc, wbasis/*1/Variance*/, &nbasis);
	}
}
//get the next action and the V value
void AmosMG::get_RBF_network_out(double* in, double* out) {
	double u_tmp1[2];
	u_tmp1[0] = 0.0;
	u_tmp1[1] = 0.0;
	ngnet->incsbox_output(&VALUE, in, u_tmp1, &nbasis);

	out[0] = u_tmp1[0];
	out[1] = clip(u_tmp1[1], -max_action, max_action);

	if (!USE_SAVED_RBF) {
		///get actual actor value
		double u_tmp[1];
		u_tmp[0] = out[1];
		//ngnet_actor->incsbox_output(&VALUE_ACTOR, x, u_tmp, &nbasis_actor);
		actor_out = u_tmp[0];

		//get noise
	#define		exploration_gain 0.999//0.9997
		double exp = robotstate.get_exploration(real_v, exploration_gain);

		bool ico_active_param = false;
		if (ico_out != 0){
			exploration_out = ico_out;
			ico_active_param = true;
		}
		else
			exploration_out = exp;

		exploration_out = clip(exploration_out, -max_action, max_action);
		out[1] = robotstate.bind_exploration(u_tmp[0], exploration_out, ico_active_param);
		actual_out = out[1];
	} else {
		exploration_out = 0;
		actor_out = out[1];
		actual_out = actor_out;
	}
}
//perform the updates to the RBF network (actor + critic)
void AmosMG::update_critic_actor(double* input_old, double td_error
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
//add sample to sample list (this is done each time step)
void AmosMG::submit_sample(double* input_old, double td_error, double pure_error, double actor_update_value) {
	Sample s;
	for (int i=0; i<4; i++)
		s.inputs.push_back(input_old[i]);
	s.tds.push_back(td_error);
	s.tds.push_back(actor_update_value);
	s.p_error = pure_error;
	samples.push_back(s);
}
