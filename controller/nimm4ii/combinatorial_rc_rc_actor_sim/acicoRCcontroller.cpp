/*************************************************************************
 * Continous actor-critic controller with Reservoir critic
 * Needs the networkmatrix.cpp and networkmatrix.h files to call the ESN network
 *
 * Created/modified by Sakyasingha Dasgupta August 13, 2013
 */

/**************************************************************************
 * Details of actor-critic learning with RBD neurons
 * Important: you need to change the following parts:
 *
 * "XDIM = number of actor inputs according to your robot sensory inputs"
 * "UDIM = number of actor output according to your robot control output"
 * "modify-> for(loop)-> init_funcapprox() according to your sensory input space"
 * "basis[_ias0] = N_IN0_RBF; change N_IN0_RBF according to your sensory input space"
 * "basis[_ias1] = N_IN0_RBF; change N_IN1_RBF according to your sensory input space"
 * "modify-> reward_function() according to your preferred reward"
 * "modify-> check_limit() according to your preferred checking state"
 *
 *
 * Poramate Manoonpong July 27, 2011
 **************************************************************************/

#include "acicoRCcontroller.h"
#include <selforg/controller_misc.h>
//#include <../Echo-State-Network/networkmatrix.h>
#include <esn-framework/networkmatrix.h>

//----AC network parameters------------//
#include "ngnet.h"
NGNet* ngnet;
Cell VALUE;

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

//-----ESN network-----//

ESNetwork * ESN, * ESN_actor;

float * ESinput;
float * EAinput;
float * EATrainOutput;
float * ESTrainOutput;

double gain0, gain1, rate_rls, roh0, roh1;

double EXP_keep_value;

//----AC network parameters------------//

using namespace matrix;
using namespace std;

bool learn_critic;
bool learn_actor;

double r_total = 0.0;

double accum_error = 0.0;

ACICOControllerV14::ACICOControllerV14(const ACICOControllerV14Conf& _conf)
: AbstractController("ACICOControllerV14", "$Id: "), conf(_conf)
{
	//added:
//
	ESN = new ESNetwork(5/*+1*/,1,100, false, false, 0, false); // try with 50,100, 200, 300 reservoir neurons

	//********************* RC Parameters ********************
	ESN->InputSparsity = 50;
	ESN->InputWeightRange = 0.5;
	ESN->LearnMode = 2;
	ESN->Loadweight = false;
	ESN->NoiseRange = 0.001;
	ESN->RCneuronNoise = true;
//	ESN->withRL = 1;

	//********************* RC Parameters ********************

	ESN->generate_random_weights(10 /*90*/, 0.95);

//	matrix::Matrix *storedweights;
//
//	storedweights = new matrix::Matrix(1,100);
//
//	//initialize store weights to null
//
//	for (int i = 0; i<100; i++)
//		for (int j = 0; j < 1; j++)
//			storedweights->val(i,j) = 0.0;



	//-----------ESN nonlinear actor--uncomment this for nonlinear actor-------------------------//
	ESN_actor = new ESNetwork(4,1,100, false, false, 0);
	ESN_actor->generate_random_weights(10 /*90*/);
	ESN_actor->leak = false;



  ESN_actor->InputSparsity = 50;
  ESN_actor->InputWeightRange = 0.5;
  ESN_actor->LearnMode = 1; // RLS
  ESN_actor->Loadweight = false;
  ESN_actor->NoiseRange = 0.001;
  ESN_actor->RCneuronNoise = false;
  ESN_actor->withRL = 2; // Supervise



//	ESN->leak = false;

	ESN->outnonlinearity = 2;//2; //2 ; //1;


	//-----------ESN nonlinear actor--uncomment this for nonlinear actor-------------------------//
	ESN_actor->outnonlinearity = 0;

	ESN->nonlinearity = 2;// 2;

	//-----------ESN nonlinear actor--uncomment this for nonlinear actor-------------------------//
	ESN_actor->nonlinearity = 2; //internal

   exploreEnd = false;

	int time;



	td_error=0.0;

	ESinput = new float[5];//sizeof(xt)];   // size of input as number of robot sensors

	//-----------ESN nonlinear actor--uncomment this for nonlinear actor-------------------------//
	EAinput = new float[4];

	cout << "ESN generated" << endl;


	/* initialize inputs to 0 */
	for(unsigned int i = 0; i < 5; i++)
	{
		ESinput[i] = 0.0;

	}

	//-----------ESN nonlinear actor--uncomment this for nonlinear actor-------------------------//
	for(unsigned int i = 0; i < 4; i++)
		{
			EAinput[i] = 0.0;

	}





	ESTrainOutput = new float[1]; // single ouput neuron
//
	ESTrainOutput[0] = 0.0;



	//-----------ESN nonlinear actor--uncomment this for nonlinear actor-------------------------//
	EATrainOutput = new float[1];
	for(unsigned int i = 0; i < 1; i++)
			{
		EATrainOutput[i] = 0.0;

			}

	preprogrammed_steering_control_active =false;
	t=0;
	//for manual steering
	mc[0]=0;
	mc[1]=0;
	mc[2]=0;
	mc[3]=0;



	//Save files
	outFileacico.open("ACICOv14.txt");
	outFiletd.open("TDdata.txt");
	outFilevtcurve.open("XYVtdata.txt");
	//	outFilevtcurveGR.open("XYVtdataR.txt");
	//	outFilevtcurveG.open("XYVtdataG.txt");
	//	outFilevtcurveB.open("XYVtdataB.txt");
	//	outFilevtcurveY.open("XYVtdataY.txt");
	outFileicolearning.open("ICOlearningcurve.txt");
	outFileiter.open("iter.txt");
	//added:
	outFileCalibrate.open("Data.txt");
	//-----AC_ICO Learning------------------//
	resetLearning_RL = 0;
	output_actor = 0.0;

	//-----Set vector size------------------//
	input_angle_s.resize(4);
	ias_w.resize(2);
	etrace_io.resize(2);
	etrace_io_old.resize(2);


	//-----Plot values----------------------//

	/*
  addInspectableValue("exploration",&exploration[0],"exploration");
	addInspectableValue("exploration_lowpass",&exploration_lowpass[0],"exploration_lowpass");
	addInspectableValue("exploration_g",&exploration_g[0],"exploration");
	addInspectableValue("exploration_g1",&exploration_g[1],"exploration1");
	addInspectableValue("exploration_lowpass_g",&exploration_lowpass_g[0],"exploration_lowpass");
	addInspectableValue("exploration_lowpass_g1",&exploration_lowpass_g[1],"exploration_lowpass1");
	addInspectableValue("exp_output0",&exp_output[0],"exploration_lowpass");
	addInspectableValue("exp_output1",&exp_output[1],"exploration_lowpass1");
	addInspectableValue("output_actor",&output_actor,"output_actor");
	addInspectableValue("input0",&input_angle_s.at(0),"input0");
	addInspectableValue("input1",&input_angle_s.at(1),"input1");
	addInspectableValue("reward0",&reward0,"reward0");
	addInspectableValue("reward1",&reward1,"reward1");
	addInspectableValue("+ias_w.at(0)",&ias_w.at(0),"ias_w.at(0)");
	addInspectableValue("-ias_w.at(1)",&ias_w.at(1),"ias_w.at(1)");
	addInspectableValue("ut[0]",&ut[0],"ut[0]");
	addInspectableValue("exploration_g",&exploration_g[0],"exploration_g");
	addInspectableValue("deri_alpha",&deri_alpha,"deri_alpha");
	addInspectableValue("alpha_old",&alpha_old,"alpha_old");
	addInspectableValue("alpha",&alpha,"alpha");
	addInspectableValue("alpha2",&alpha2,"alpha2");
	addInspectableValue("alpha3",&alpha3,"alpha3");
	addInspectableValue("alpha4",&alpha4,"alpha4");
	addInspectableValue("ir_l",&ir_l,"ir_l");
	addInspectableValue("ir_r",&ir_r,"ir_r");
	addInspectableValue("irl_lowpass ",&irl_lowpass ,"irl_lowpass");
	addInspectableValue("irr_lowpass ",&irr_lowpass ,"irr_lowpass");
	addInspectableValue("input_ir_s",&input_ir_s,"input_ir_s");
	addInspectableValue("xt[2]_l",&xt[2],"&xt[2]");
	addInspectableValue("xt[3]_r",&xt[3],"&xt[3]");
	addInspectableValue("xt_reflex_irl",&xt_reflex_irl,"xt_reflex_irl");
	addInspectableValue("xt_reflex_irr ",&xt_reflex_irr,"xt_reflex_irr");
	addInspectableValue("deri_reflex_irl",&deri_reflex_irl,"xt_reflex_irl");
	addInspectableValue("deri_reflex_irr",&deri_reflex_irr,"xt_reflex_irr");
	addInspectableValue("u_ico[0]",&u_ico[0],"u_ico[0]");
	addInspectableValue("u_ico[1]",&u_ico[1],"u_ico[1]");
	addInspectableValue("exp_output[0]",&exp_output[0],"exp_output[0]");
	addInspectableValue("exp_output[1]",&exp_output[1],"exp_output[1]");
	addInspectableValue("xt_reflex_angle",&xt_reflex_angle,"xt_reflex_angle");
	addInspectableValue("xt_reflex_angle_old",&xt_reflex_angle_old,"xt_reflex_angle_old");
	addInspectableValue("distance",&distance,"distance");
	addInspectableValue("xt_ico[_ias0]",&xt_ico[_ias0],"xt_ico[_ias0]");
	addInspectableValue("input_distance_s",&input_distance_s,"input_distance_s");
	addInspectableValue("deri_xt_reflex_angle",&deri_xt_reflex_angle,"deri_xt_reflex_angle");
	addInspectableValue("k_ico[0]",&k_ico[0],"k_ico[0]");
	addInspectableValue("k_ico[1]",&k_ico[1],"k_ico[1]");

	addInspectableValue("input_angle_s.at(0)",&input_angle_s.at(0),"input_angle_s.at(0)");
	addInspectableValue("input_angle_s.at(1)",&input_angle_s.at(1),"input_angle_s.at(1)");
	addInspectableValue("input_angle_s.at(2)",&input_angle_s.at(2),"input_angle_s.at(2)");
	addInspectableValue("input_angle_s.at(3)",&input_angle_s.at(3),"input_angle_s.at(3)");
	addInspectableValue("xt_ico_lowpass",&xt_ico_lowpass,"xt_ico_lowpass");
	addInspectableValue("xt_ico_lowpass2",&xt_ico_lowpass2,"xt_ico_lowpass2");
	addInspectableValue("xt_ico_lowpass3",&xt_ico_lowpass3,"xt_ico_lowpass3");
	addInspectableValue("xt_ico_lowpass4",&xt_ico_lowpass4,"xt_ico_lowpass4");

	addInspectableValue("input_distance_s",&input_distance_s,"input_distance_s");
	addInspectableValue("input_distance_s2",&input_distance_s2,"input_distance_s2");
	addInspectableValue("input_distance_s3",&input_distance_s3,"input_distance_s3");
	addInspectableValue("input_distance_s4",&input_distance_s4,"input_distance_s4");

	addInspectableValue("exp_output[0]",&exp_output[0] ,"exp_output[0]");
	addInspectableValue("xt_reflex_angle",&xt_reflex_angle,"xt_reflex_angle");
	addInspectableValue("xt_ico[_ias0]",&xt_ico[_ias0],"xt_ico[_ias0]");
	addInspectableValue("xt_ico[_ias1]",&xt_ico[_ias1],"xt_ico[_ias1]");
	addInspectableValue("k_ico[0]",&k_ico[0],"k_ico[0]");
	addInspectableValue("k_ico[1]",&k_ico[1],"k_ico[1]");
	addInspectableValue("k_ico[2]",&k_ico[2],"k_ico[2]");
	addInspectableValue("u_ico_in[0]",&u_ico_in[0],"u_ico_in[0]");
	addInspectableValue("u_ico_in[1]",&u_ico_in[1],"u_ico_in[1]");
	addInspectableValue("u_ico_in[2]",&u_ico_in[2],"u_ico_in[2]");

	addInspectableValue("deri_xt_reflex_angle2",&deri_xt_reflex_angle2,"deri_xt_reflex_angle2");
	addInspectableValue("deri_xt_reflex_angle3",&deri_xt_reflex_angle3,"deri_xt_reflex_angle3");
	 */
	addInspectableValue("VT",&Vt,"Vt");
	addInspectableValue("rt",&rt,"rt");
	addInspectableValue("TD_Error",&td_error,"TD_Error");
	addInspectableValue("acum_reward",&acum_reward,"acum_reward");
	addInspectableValue("exploration",&exploration[0],"exploration");
	addInspectableValue("exploration_lowpass",&exploration_lowpass[0],"exploration_lowpass");
	addInspectableValue("exploration_g",&exploration_g[0],"exploration");
	addInspectableValue("exploration_g1",&exploration_g[1],"exploration1");
	addInspectableValue("exploration_lowpass_g",&exploration_lowpass_g[0],"exploration_lowpass");
	addInspectableValue("exploration_lowpass_g1",&exploration_lowpass_g[1],"exploration_lowpass1");
	addInspectableValue("xt[_ias0]",&xt[_ias0],"xt[_ias0]");
	addInspectableValue("irl_lowpass",&irl_lowpass,"irl_lowpass");

	//----Protecting Reset parameters------//
	initialized = false; //-----------------------------------------------(FRANK)
	use_Learned_weightsv2 = false;//--------------true-----------------------------------------------TEST
	large_input_rangev2 = false;

	//No learning noise
	no_learning_noise = true ; //true;
	//----Protecting Reset parameters------//

};


ACICOControllerV14::~ACICOControllerV14()
{
	//Save files
	outFileacico.close();
	outFiletd.close();
	outFilevtcurve.close();
	outFileicolearning.close();
	outFileiter.close();
	//	outFilevtcurveGR.close();
	//	outFilevtcurveG.close();
	//	outFilevtcurveB.close();
	//	outFilevtcurveY.close();
	//added:
	outFileCalibrate.close();

	//added


	//----- ESN objects garbage collection ---- //

	delete []ESN;
	delete []ESinput;
	delete []ESTrainOutput;

	delete []ESN_actor;
	delete []EAinput;
	delete []EATrainOutput;

	cout << "ESN unloaded" << endl;


}




void ACICOControllerV14::init(int sensornumber, int motornumber, RandGen* randGen)
{
	stepnumber_t = 0;
	if(!initialized) //-----------------------------------------------(FRANK)
	{
		roh0 = 0.01;//1000;//0.001;//pow(10,5);
		roh1 = 0.01; //1000;//0.001;//pow(10,5);

		rate_rls = 0.9998; //0.99998; //0.995;

		w_ico = 0.5; //0.1;//0.5; //0.5;
		w_ac= 0.5; //0.1;//0.5; //0.5;
		rate = 0.0005;
		rate_ss = 0.000005; // 0.0005

		ss_rate = 0.0005;

		//Change between ESN and RBF critic
		ESN_critic = true; //true; //false; //true; //true; //true; //true; //false; //true;//true;

		//turn on for POMDP scenario (for fully observable case equal weghtage works best)
		learn_combined_weights = true; //false ; //true ;//true; //true ; //true /*true*/;

		scenario_flag = 1; //2;//1  // default case full observable 1; set to 2 for POMDP

		reduce_noise = 0;
		reduce_noise_ICO = 0;

		start_time = time(NULL);


		number_sensors = sensornumber;
		number_motors = motornumber;

		//  cout<<"sensor number = "<<sensornumber;

		old_state=0;
		old_action=0;


		//	// no brake,
		//	// brake 1, brake 2, brake 3, brake 4,
		//	// brake 1 and 2, brake 1 and 3, brake 1 and 4,
		//	// brake 2 and 3, brake 2 and 4, brake 3 and 4,
		//	// brake 1,2,3, brake 1,2,4, brake 1,3,4,  brake 2,3,4
		//	// brake 1,2,3,4
		//qlearner = new QLearning(number_states, number_actions);


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
		xdim = XDIM /*=2 inputs, Set for different system */;
		udim = 1;/*output of critic always = 1, Set for different system */;
		init_funcapprox();

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

		//Adding initial bias to let the robot reacts to obstacle
		k[2]= 0.5;
		k[3]= 0.5;

		//		double MAX_k0, MIN_k0;
		//		double MAX_k1, MIN_k1;
		//
		//		MAX_k0 = 1.0;
		//		MIN_k0 = -1.0;
		//		MAX_k1 = 1.0;
		//		MIN_k1 = -1.0;
		//
		//		k[0] = ((MAX_k0-MIN_k0)*((float)rand()/RAND_MAX))+MIN_k0;
		//		k[1] = (((MAX_k1-MIN_k1)*((float)rand()/RAND_MAX))+MIN_k1)*0.5;



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


		//Meta parameters of learning
		//Exploration parameters (Noise amplitude for exploration)
		//IF input range very small e.g., 0,..,3.414 --> use small si~2.0
		//IF input range very large e.g., 0,..,180 --> use larger si~10.0
		if(large_input_rangev2)
		{
			si = 10.0;
			rate_policy = 0.5;//0.9;//0.1;//0.5; // learning rate of actor
			rate_valuefunction = 0.5;//0.9;//0.1;//0.5;// learning rate_critic of weights from hidden RBF neurons to its output neuron (critic)
			printf(" Large INPUT range [0,..,180]-> si = %f, rate_policy = %f, rate_valuefunction = %f \n\n", si, rate_policy, rate_valuefunction);
		}
		else
		{
			si = 1.0;//10.0;
			rate_policy = 0.001; //0.001;//0.01;//0.01;// learning rate of actor
			rate_valuefunction = 0.7;// learning rate_critic of weights from hidden RBF neurons to its output neuron (critic)
			printf("Small INPUT range [-pi/2,..,pi/2]-> si = %f, rate_policy = %f, rate_valuefunction = %f \n\n", si, rate_policy, rate_valuefunction);
		}



		//--CRITIC
		td_limit = 1.0; // limit of clipping TDerror (critic)
		gam = 0.98;//0.98;//0.98; // discount factor of Vt (critic)
		error_thresh = 0.1; // threshold for critic weight changes
		unit_activation_thresh = 0.3;//0.3;//0.6; // threshold for critic weight changes
		lambda_v = 0.0;//0.9;//0.7;// E trace of Vt value: 0.0<= lambda_v < 1.0, if lambda_v ~> 1.0 = hqigh low pass, if lambda_v ~> 0.0 = no low pass (critic)
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
		rate_ico = 0.001; //0.001; //0.005;   // 0.005;// ICO         0.001; // ICO_AC
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

	}

}
void ACICOControllerV14::calculateAnglePositionFromSensors(const sensor* x_) //as it says on the tin
{

	//Goal 1 // red
	alpha_old =  alpha;

	if (sign(x_[12 /*10 x*/]/*X axis*/)>0)
	{ // goal is in front
		//alpha = atan(x_[11]/x_[10]) * 180 / M_PI; // angle in degrees
		alpha = atan(x_[13/*11 y*/]/x_[12 /*10 x*/]) * 180 / M_PI; // angle in degrees
	}
	else
	{ // behind robot angles "continue", directly behind robot is 180 degrees, left is negative, right is positive
		alpha_tmp = -1*atan (x_[13/*11 y*/]/x_[12 /*10 x*/]) * 180 / M_PI; // angle in degrees
		if (alpha_tmp<=0)
		{ // left
			alpha = (-90 + (-90-alpha_tmp));
		}
		else
		{ // right
			alpha = ( 90 + ( 90-alpha_tmp));
		}
	}

	input_angle_s.at(0) =  alpha/180.*M_PI; // map sensor input to +-pi

	//Goal 2 // Green
	alpha_old2 =  alpha2;

	if (sign(x_[15 /*x*/]/*X axis*/)>0)
	{ // goal is in front
		alpha2 = atan(x_[16/*y*/]/x_[15 /*x*/]) * 180 / M_PI; // angle in degrees
	}
	else
	{ // behind robot angles "continue", directly behind robot is 180 degrees, left is negative, right is positive
		alpha_tmp2 = -1*atan (x_[16/*y*/]/x_[15/*x*/]) * 180 / M_PI; // angle in degrees
		if (alpha_tmp2<=0)
		{ // left
			alpha2 = (-90 + (-90-alpha_tmp2));
		}
		else
		{ // right
			alpha2 = ( 90 + ( 90-alpha_tmp2));
		}
	}


	input_angle_s.at(1) =  alpha2/180.*M_PI; // map sensor input to +-pi

	//Goal 3 // Blue
	alpha_old3 =  alpha3;

	if (sign(x_[18 /*x*/]/*X axis*/)>0)
	{ // goal is in front
		alpha3 = atan(x_[19/*y*/]/x_[18 /*x*/]) * 180 / M_PI; // angle in degrees
	}

	else
	{ // behind robot angles "continue", directly behind robot is 180 degrees, left is negative, right is positive
		alpha_tmp3 = -1*atan (x_[19/*y*/]/x_[18/*x*/]) * 180 / M_PI; // angle in degrees
		if (alpha_tmp3<=0)
		{ // left
			alpha3 = (-90 + (-90-alpha_tmp3));
		}
		else
		{ // right
			alpha3 = ( 90 + ( 90-alpha_tmp3));
		}
	}

	input_angle_s.at(2) =  alpha3/180.*M_PI; // map sensor input to +-pi

	//Goal 4 // Yellow
	alpha_old4 =  alpha4;

	if (sign(x_[21 /*x*/]/*X axis*/)>0)
	{ // goal is in front
		alpha4 = atan(x_[22/*y*/]/x_[21 /*x*/]) * 180 / M_PI; // angle in degrees
	}
	else
	{ // behind robot angles "continue", directly behind robot is 180 degrees, left is negative, right is positive
		alpha_tmp4 = -1*atan (x_[22/*y*/]/x_[21/*x*/]) * 180 / M_PI; // angle in degrees
		if (alpha_tmp4<=0)
		{ // left
			alpha4 = (-90 + (-90-alpha_tmp4));
		}
		else
		{ // right
			alpha4 = ( 90 + ( 90-alpha_tmp4));
		}
	}


	input_angle_s.at(3) =  alpha4/180.*M_PI; // map sensor input to +-pi

}

void ACICOControllerV14::calculateDistanceToGoals(const sensor* x_)
{
	max_dis = position_x_robot*position_x_robot; //normalize
	distance = (sqrt(pow(x_[13/*11 y*/],2)+pow(x_[12/*10 x*/],2)));
	input_distance_s = distance/100;//85;//max_dis;

	max_dis2 = position_x_robot*position_x_robot; //normalize
	distance2 = (sqrt(pow(x_[16/*y*/],2)+pow(x_[15/*x*/],2)));
	input_distance_s2 = distance2/100;//85;//max_dis;

	max_dis3 = position_x_robot*position_x_robot; //normalize
	distance3 = (sqrt(pow(x_[19/*y*/],2)+pow(x_[18/*x*/],2)));
	input_distance_s3 = distance3/100;//85;//max_dis;

	max_dis4 = position_x_robot*position_x_robot; //normalize
	distance4 = (sqrt(pow(x_[22/*y*/],2)+pow(x_[21/*x*/],2)));
	input_distance_s4 = distance4/100;//85;//max_dis;
}

void ACICOControllerV14::inputSensorSignals(const sensor* x_)
{
	xt_ico[_ias0] = input_angle_s.at(0);

	xt_ico[_ias1] = input_distance_s;
	xt_ico[_ias2] = irl_lowpass*1.5;
	xt_ico[_ias3] = irr_lowpass*1.5;

	xt_ico2[_ias0] = input_angle_s.at(1);

	xt_ico2[_ias1] = input_distance_s2;
	xt_ico2[_ias2] = irl_lowpass*1.5;
	xt_ico2[_ias3] = irr_lowpass*1.5;

	xt_ico3[_ias0] = input_angle_s.at(2);

	xt_ico3[_ias1] = input_distance_s3;
	xt_ico3[_ias2] = irl_lowpass*1.5;
	xt_ico3[_ias3] = irr_lowpass*1.5;


	xt_ico4[_ias0] = input_angle_s.at(3);

	xt_ico4[_ias1] = input_distance_s4;
	xt_ico4[_ias2] = irl_lowpass*1.5;
	xt_ico4[_ias3] = irr_lowpass*1.5;

	xt_ir = input_ir_s;

	//Lowpass filters of angle signals
	double gain_r = 0.0;//0.99;

	xt_ico_lowpassold = xt_ico_lowpass;
	xt_ico_lowpass2old = xt_ico_lowpass2;
	xt_ico_lowpass3old = xt_ico_lowpass3;
	xt_ico_lowpass4old = xt_ico_lowpass4;

	xt_ico_lowpass = (1-gain_r)*xt_ico[_ias0]+xt_ico_lowpassold*gain_r;
	xt_ico_lowpass2 = (1-gain_r)*xt_ico2[_ias0]+xt_ico_lowpass2old*gain_r; //*** USED //Goal Green 2
	xt_ico_lowpass3 = (1-gain_r)*xt_ico3[_ias0]+xt_ico_lowpass3old*gain_r; //*** USED //Goal Blue 3
	xt_ico_lowpass4 = (1-gain_r)*xt_ico4[_ias0]+xt_ico_lowpass4old*gain_r;

}

void ACICOControllerV14::calculateAngleReflexSignals(const sensor* x_)
{
	//Angle reflex signals
	//1) goal 1
	xt_reflex_angle_old = xt_reflex_angle;

	if(xt_ico[_ias1] < range_reflex/*1.2 ~0.0120 very close to target*/)
	{
		//xt_reflex_angle = xt_ico_lowpass;//input_angle_s.at(0);
		xt_reflex_angle = input_angle_s.at(0);
	}
	else
	{
		xt_reflex_angle = 0.0;
	}

	deri_xt_reflex_angle = xt_reflex_angle-xt_reflex_angle_old;
	//deri_xt_reflex_angle = clip(deri_xt_reflex_angle, 0, 1.0);

	//2) goal 2
	xt_reflex_angle_old2 = xt_reflex_angle2;

	if(xt_ico2[_ias1] < range_reflex/*1.2 ~0.0120 very close to target*/)
	{
		//xt_reflex_angle2 = xt_ico_lowpass2;//input_angle_s.at(1);
		xt_reflex_angle2 = input_angle_s.at(1);
	}
	else
	{
		xt_reflex_angle2 = 0.0;
	}

	deri_xt_reflex_angle2 = xt_reflex_angle2-xt_reflex_angle_old2;
	//deri_xt_reflex_angle2 = clip(deri_xt_reflex_angle2, 0, 1.0);


	//3) goal 3
	xt_reflex_angle_old3 = xt_reflex_angle3;

	if(xt_ico3[_ias1] <range_reflex/*1.2 ~0.0120 very close to target*/)
	{
		//xt_reflex_angle3 = xt_ico_lowpass3;//input_angle_s.at(2);
		xt_reflex_angle3 = input_angle_s.at(2);
	}
	else
	{
		xt_reflex_angle3 = 0.0;
	}

	deri_xt_reflex_angle3 = xt_reflex_angle3-xt_reflex_angle_old3;
	//deri_xt_reflex_angle3 = clip(deri_xt_reflex_angle3, 0, 1.0);


	//4) goal 4
	xt_reflex_angle_old4 = xt_reflex_angle4;

	if(xt_ico4[_ias1] <range_reflex/*1.2 ~0.0120 very close to target*/)
	{
		//xt_reflex_angle4 = xt_ico_lowpass4;//input_angle_s.at(3);
		xt_reflex_angle4 = input_angle_s.at(3);
	}
	else
	{
		xt_reflex_angle4 = 0.0;
	}

	deri_xt_reflex_angle4 = xt_reflex_angle4-xt_reflex_angle_old4;
	//deri_xt_reflex_angle4 = clip(deri_xt_reflex_angle4, 0, 1.0);


	//IR reflex signals
	if(xt_ir>0.7)
		xt__reflex_ir = 1.0;
	if(xt_ir<-0.7)
		xt__reflex_ir = -1.0;  //reflex signal
	if(xt_ir>-0.7 && xt_ir<0.7)
		xt__reflex_ir = 0.0;
}


void ACICOControllerV14::step(const sensor* x_, int number_sensors, motor* y_, int number_motors){

	u_ico_old = u_ico[0]*w_ico;
	ut_old = ut[0]*w_ac;

	double scale = 0.5;// 1.0;

	bool combinecontrol = true; //true; //false ; //true; // if set combine = true ,  actor_critic_control and ico_control  must be true
	// if set combine = false ,  actor_critic_control or ico_control  must be true

	bool manual_control = false;
	bool actor_critic_control = true; //true; // true;
	bool ico_control = true; //true ;//true; // false; //true;

	//  position_robot = getPosition();

	/*
  //static int ntrial = 0; //ANOTHER OPTION!!!

  // >> i/o operations here <<
  //outFileacico<<reset_learning<<' '<<reset_learning<<endl;

	 ******************************************************************************
	 * Sensory preprocessing of 13 Sensory Inputs
	 ******************************************************************************

	//x_[0] = wheel velocity left front sensor
	//x_[1] = wheel velocity right front sensor
	//x_[2] = wheel velocity left rear sensor
	//x_[3] = wheel velocity right rear sensor

	//RIGHT IR
	//x_[4] = IR front right
	//x_[11] = IR side front right wheel
	//x_[10] = IR side rear right wheel

	//LEFT IR
	//x_[5] = IR front left
	//x_[7] = IR side front left wheel
	//x_[6] = IR side rear left wheel

	//x_[8] = IR back left
	//x_[9] = IR back right

	//Goal 1 //Red
	//x_[12] = relative position to the ball X
	//x_[13] = relative position to the ball Y
	//x_[14] = relative position to the ball Z not used

	//Goal 2 //Green
	//x_[15] = relative position to the ball X
	//x_[16] = relative position to the ball Y
	//x_[17] = relative position to the ball Z not used

	//Goal 3 //Blue
	//x_[18] = relative position to the ball X
	//x_[19] = relative position to the ball Y
	//x_[20] = relative position to the ball Z not used

	//Goal 4 //yellow
	//x_[21] = relative position to the ball X
	//x_[22] = relative position to the ball Y
	//x_[23] = relative position to the ball Z not used */

	//---1) Calculating angle position from goals

	calculateAnglePositionFromSensors(x_);

	//---2) Calculating inputs to leaner neuron

	deri_alpha = alpha - alpha_old;
	deri_alpha = clip(deri_alpha, -td_limit /*-1*/, td_limit /*1*/); /// CLIP limit TD error

	//---3) Calculating distance to the goals 1-4

	calculateDistanceToGoals(x_);

	//---4) Calculating IR sensors for obstacle detection
	ir_l = (x_[5]+x_[7]+x_[6]);//(x_[5]+x_[6]+x_[7]); // +
	ir_r = (x_[4]+x_[11]+x_[10]);//(x_[4]+x_[8]+x_[9]); // -

	input_ir_s = ir_l-ir_r;

	//preprocessing using a low pass filter
	double gain_ir = 0.95;//0.99;

	irl_lowpass_old = irl_lowpass;
	irr_lowpass_old = irr_lowpass;

	irl_lowpass = (1-gain_ir)*ir_l+irl_lowpass_old*gain_ir;
	irr_lowpass = (1-gain_ir)*ir_r+irr_lowpass_old*gain_ir;

	irl_back = x_[8];
	irr_back = x_[9];


	///----------Sensor input-----------------------------------//
	/*
  //ANGLE FROM GOALS
  //input_angle_s.at(0) = Goal 1 red  [-1(-2),...,1(2)]
  //input_angle_s.at(1) = Goal 2 green [-1(-2),...,1(2)]
  //input_angle_s.at(2) = Goal 3 blue [-1(-2),...,1(2)]
  //input_angle_s.at(3) = Goal 4 yellow [-1(-2),...,1(2)]

  //DISTANCE TO GOALS
  //input_distance_s  = normalized distance from Goal 1 [0,...,1]
  //input_distance_s2 = normalized distance from Goal 2 [0,...,1]
  //input_distance_s3 = normalized distance from Goal 3 [0,...,1]
  //input_distance_s4 = normalized distance from Goal 4 [0,...,1]
  //distance, distance2, distance3, distance4 = from goal 1,2,3,4

  //LEFT, RIGHT IR sensors
  //irl_lowpass*1.5;  = average IR on left
  //irr_lowpass*1.5;  = average IR on  right
  //irl_back = back on left
  //irr_back = back on right

	 ******************************************************************************
	 * Sensor processing for ICO and RL
	 *******************************************************************************/
	//1) Predictive Sensor processing

	//Inputs: Angle and distance sensors
	inputSensorSignals(x_);

	//2) Reflex Sensor processing

	calculateAngleReflexSignals(x_);


	/*******************************************************************************
	 * Motor control and learning
	 *******************************************************************************/
	// here begins actor critic learning
	if(actor_critic_control)
	{
		//printf("\n position_x_robot = %d gam = %f, si = %f th_exp_td_vt= %f time_to_test_learned_policy = %f \n\n",position_x_robot, gam, si,th_exp_td_vt,time_to_test_learned_policy);
		//----AC network parameters------------//
		static int iter = 0; // Initial only one time
		static int iter_old = 0;

		static int trial_n = 0;

		static double Vt_old = 0.0;// Initial only one time
		//td_error = 0.0;

		double  rt_reducenoise;
		//----AC network parameters------------//

		iter_old = iter;

		/*****SENSORY INPUTS*******************/

		switch (scenario_flag) {

		case 1:
		{
			//Green
			xt[_ias0] = xt_ico_lowpass2;// input to controller Set sensory signal of robot (State) to actor input xt

			//Blue
			xt[_ias1] = xt_ico_lowpass3;

			xt[_ias2] = irl_lowpass*1.5;

			xt[_ias3] = irr_lowpass*1.5;

			break;

			std::cout<<"\n I am here \n";
		}

		case 2:
		{
			/* partial observable case */

			xt[_ias2] = irl_lowpass*1.5;

			xt[_ias3] = irr_lowpass*1.5;


			if (xt_ico2[_ias1] <= 0.8 || xt_ico3[_ias1] <=0.8/*&& xt_ico2[_ias1] > 0.5*/)
				{xt[_ias0] = xt_ico_lowpass2 ;// input to controller Set sensory signal of robot (State) to actor input xt

				}
			else
				{xt[_ias0] = 0.0 ; //xt_ico_lowpass2;
				}
			//Blue
			if(xt_ico3[_ias1] <=0.8 || xt_ico2[_ias1] <= 0.8/*&& xt_ico3[_ias1] > 0.5*/ )
				xt[_ias1] = xt_ico_lowpass3; //0.0;

			else
				xt[_ias1] = 0.0 ; //xt_ico_lowpass3;

			break;

		}

		}

		//IF REACH GOAL
		if(/*xt_ico[_ias1] < 0.03 ||*/ xt_ico2[_ias1] < 0.05 /*|| xt_ico3[_ias1] < 0.05 || xt_ico4[_ias1] < 0.03*/)
			reduce_noise++;

		//ADD
		rt_reducenoise = reward_function(xt, ut);
		if(rt_reducenoise == -1.0)
			reduce_noise--;

		if(reduce_noise<0)
			reduce_noise = 0;
		//ADD

		exp_output_decay = exp(-reduce_noise/200);//100);

		//printf("\n rt %f \n nrepeat: %d \n exp_output_decay:  %f reduce_noise: %d\n  xt_ico2[_ias1] %f  : xt_ico3[_ias1] %f \n",rt, nrepeat,exp_output_decay, reduce_noise,  xt_ico2[_ias1],  xt_ico3[_ias1]);



		/*****AFTER Fail->RESET*******************/
		if(failure_flag || resetLearning_RL == 0 /*when approach goal or hit obstacles*/)
		{


			//----Important for learning-----//
			//Robot will be reset after this by simulation//
			failure_flag = 0;
			Vt_old = 0.0; // RESET p_old or reset Vt_old = V1 // as expected Max

			//----Important for learning-----//

			ntrial++;
			iter = 0;
		//	acum_reward = 0.0; // RESET acc reward // may need to analyze or evaluate system

			accum_error = 0.0;
			sig_out_log[ntrial] = sig_out[0]; // may need to analyze or evaluate system
			resetLearning_RL++;

//			ESN->takeStep(ESTrainOutput, 0.0055/*1.5*//*1.8*/, 1, true/*learn_critic*/, 0);
//
//			Vt = ESN->outputs->val(0, 0) * 50;

//
//			for(int i = 0; i < ESN->networkNeurons; i++)
//				for(int j=0;j< 1; j++)
//				{
//					ESN->intermediates->val(i,j)= 0.0;
//				}
////////////
//			for(int i = 0; i < ESN_actor->networkNeurons; i++)
//							for(int j=0;j< 1; j++)
//							{
//								ESN_actor->intermediates->val(i,j)= 0.0;
//							}


		//	acum_reward = 0.0;

		}



		/*** LEARNING STATE*******************/
		if(ntrial < MAX_TRIAL)// && distance>d_threshold)
		{

			/*---(1) Output from current policy--------------*/
			old_ut = ut[0]; //*k[0];
		//	old_ut1 = ut[0]*k[1];
//			old_ut2 = ut[0]*k[2];
//			old_ut3 = ut[0]*k[3];;

			output_policy(xt /*in*/, ut /*steering control, return*/);
			ut[0] = clip(ut[0], -1.0, 1.0); /// CLIP limit ut0


			//ut[1] = clip(ut[1], -1.0, 1.0); /// CLIP limit ut1

			/*---(2) Move robot------------------------------*/



			y_ac[0] =  scale*1+ut[0];//ut[0]; // left front wheel
			y_ac[1] = scale*1-ut[0];//ut[1]; // right front wheel
			y_ac[2] = scale*1+ut[0];//ut[0]; // left rear wheel
			y_ac[3] = scale*1-ut[0];//ut[1]; // right rear wheel

			/*---(3) Cal reward-----------------------------*/

			rt = reward_function(xt, ut);

			//acum_reward += rt;

			//acum_reward = rt + pow(gam,iter)*rt;

			acum_reward = (1-0.0022)*acum_reward + 0.0022*rt;
		//	acum_reward = (1-0.07)*acum_reward + 0.07*rt;
			//with discount
			//acum_reward += pow(gam,iter)*rt;
			//printf("rt =  %f   \n\n\n acum_reward = %f \n\n\n xt_ico2: %f xt_ico3: %f \n\n", rt, acum_reward, xt_ico2[_ias1], xt_ico3[_ias1]);



			/*---(4) Check state ---------------------------*/
			failure_flag = check_limit(xt);

			//failure_flag =  check_limit(xt, irr_back, irl_back);


			/*---(5) Value function estimation--------------*/
			if (!ESN_critic)
				Vt = value_function(xt/*inputs*/); //Replaced by ESN

			// *********** commented ESN ****************
			if (ESN_critic){

				//*******************

				//*****************************

				ESTrainOutput[0]= acum_reward; //Vt_old; //acum_reward;//rt;//acum_reward;


				ESinput[0] = xt[_ias0] ; //+ gauss();
				ESinput[1] = xt[_ias1] ; //+ gauss();

				ESinput[2] = rt;

			//	if(ESinput[0] == 0.0) ESinput[0] = gauss();
			//	if(ESinput[1] == 0.0) ESinput[1] = gauss();

				ESinput[3] = xt[_ias2];
				ESinput[4] = xt[_ias3];
//				if (scenario_flag == 2)
//				{
//					if (xt[_ias0] == 0) ESinput[0] = 0.1 ;
//					if (xt[_ias1]== 0) ESinput[1] = 0.1;
//
//				}
			 //   ESinput[3] = ut[0];  // Feedback output to reservoir (not_ needed)


				if (!ESN) cout<< "critical failure: ESN not loaded" <<std::endl;

				ESN->setInput(ESinput, 5 /*3*/);// sizeof(xt)/*+1*/


			   bool learn = true;
//				bool learn = false;

		 		//if (rt !=0) learn = true;

				//if (rt >0) learn = true;

				if(exp_output[0]< 0.0001) learn = false;
//
//				else
//					learn_critic = true;

        //        accum_error += (td_error);

				ESN->takeStep(ESTrainOutput, 0.00022 /*0.00055/*0.0005*/ /*0.0055*//*1.5*//*1.8*/, td_error, learn/*learn_critic*/, 0);


				//    std::cout<< "output: " << ESN->outputs->val(0, 0) <<std::endl;

				Vt = ESN->outputs->val(0, 0) * 50; //*10; //50; //* 50  ; // * 20;

                std::cout<<"acum_reward "<<acum_reward<<"\n";


				//Vt = clip(Vt, -3, 3);
			}
			// *********** commented ESN ****************

			std::cout<<"vt = "<<Vt<<endl;
			std::cout<<"irl_lowpass"<<irl_lowpass<<std::endl;
			std::cout<<"irr_lowpass"<<irr_lowpass<<std::endl;



			/*---(6) TD error-------------------------------*/

			if(resetLearning_RL<time_to_test_learned_policy)
			{
				for(int i=0;i<UDIM;i++)
				{
					exp_output[i] = 0.0;
				}
			}


			if(failure_flag || iter == 0) // if fail (reset)
			{
				// Because Vt expected > 0 then we set Vt_old = Vt after reset and also Td error = 0

				//	acum_reward=0;
				td_error = 0.0;//rt- Vt_old;//gam*Vt - Vt_old;
				Vt_old =   Vt; //gam*Vt;//Vt; // = Vt or may be smaller NOT Important!!!!
				//printf("acum_reward = %f  rt = %f  td_error = %f Vt = %f : gam*Vt = %f, Vt_old = %f\n", acum_reward, rt, td_error, Vt, gam*Vt,Vt_old);
				//   td_error_old = td_error;
				td_error = clip(td_error, -td_limit /*-1*/, td_limit /*1*/); /// CLIP limit TD error
			}
			else
			{
				//td_error = rt + gam*Vt - Vt_old;
				td_error = rt - acum_reward + Vt - Vt_old;

				//     td_error_old = td_error;
				td_error = clip(td_error, -td_limit /*-1*/, td_limit /*1*/); /// CLIP limit TD error
			}








			//	printf("Save1 acum_reward = %f   rt = %f  td_error = %f Vt = %f : gam*Vt = %f, Vt_old = %f\n", acum_reward, rt, td_error, Vt, gam*Vt, Vt_old);
			outFiletd<<td_error<<' '<<rt<<' '<<Vt<<' '<<exp_output[0]<<' '<<k[0]<<' '<<k[1]<<' '<<k[2]<<' '<<k[3]<<' '<<nrepeat<<' '<<k_ico[1]<<' '<<k_ico[2]<<' '<<xt[_ias0]<<' '<<xt[_ias1]<<' '<<xt[_ias2]<<' '<<xt[_ias3]<<' '<<ut[0]<<' '<<u_ico[0]<<' '<<w_ico<<' '<<w_ac<<' '<<rate<<' '<<rate_ss<<' '<<rate_policy<<' '<<y_[0]<<' '<<y_ac[0]<<' '<<y_ico[0]<<endl;
			outFilevtcurve<<x_[12 /* x*/]<<' '<<x_[13/*y*/]<<' '<<Vt<<endl; //actual robot positions

			//			outFilevtcurveGR<<x_[12 /* x*/]<<' '<<x_[13/*y*/]<<' '<<Vt<<endl; // R
			//			outFilevtcurveG<<x_[15 /* x*/]<<' '<<x_[16/*y*/]<<' '<<Vt<<endl; // G
			//			outFilevtcurveB<<x_[18 /* x*/]<<' '<<x_[19/*y*/]<<' '<<Vt<<endl; // B
			//			outFilevtcurveY<<x_[21/* x*/]<<' '<<x_[22/*y*/]<<' '<<Vt<<endl; // Y

			//save iter



			Vt_old = Vt;

			//deri_td_error = td_error- td_error_old; //NOT USED
			//td_error_old = td_error;

			printf("exp_output[0] = %f : td_error = %f : output_grad[0] = %f : iter = %d \n\n: repeat= %d  k[0] = %f  k[1] = %f k[2] = %f k[3] = %f \n w_ico=%f \n w_ac = %f \n u_ac = %f \n u_ico = %f \n xt[_ias0]= %f \n xt[_ias1]= %f \n k_ico[1] = %f\n k_ico[2]= %f \n ",exp_output[0], td_error, output_grad[0], iter, nrepeat, k[0], k[1], k[2], k[3], w_ico,w_ac,ut[0],u_ico[0],xt[_ias0],xt[_ias1],k_ico[1],k_ico[2]);
			//printf("ut0 %f   \n k[0]= %f () \n k[1]= %f ()  \n", ut[0],  k[0], k[1]);


			/*---(7) Gradient method to reduce exploration-----------*/
			/* gradients for policy parameters --> return  --> sig_out -->sig--> exploration-> output = u +exploration !*/
			/* this is to reduce exploration as soon as TDerror --> ~ 0 */
			exploration_grad(exp_output /*output_policy()*/, sig_elig, sig /*exploration_output()*/, si /*amplitude->set, e.g., 10*/);

			/*---(8) Clip output gradient -for update_policy_trace()-*/


			for(int i=0;i<UDIM;i++)
			{
				//Morimoto
				//output_grad[i] = clip(exp_output[i]/(sig[i]*sig[i]),-1.0, 1.0); // with noise learning

				//Kimura & no learning noise version
				output_grad[i] = clip(exp_output[i],-1.0, 1.0); // without noise learning
			}


			/*---(9) Update parameters for value function approximator weights-*/
			if (!ESN_critic){
			  update_valuefunction_trace(xt); //return --> curr->dw = etrace of all softmax RBF hidden neurons (curr->softac);
			  update_valuefunction(xt/*input*/, td_error/*TDerror*/,rate_valuefunction);
			}
			/*---(10) Update parameters for policy weights-------------*/
			update_policy_trace(xt,lambda_p);
			//update_policy(rt, rate_policy);	//direct reward learning also work!!!
			update_policy(td_error, rate_policy); // Actor-critic learning

			if(resetLearning_RL == 1)
				checkreset = 10;
			else
				checkreset = 0;


			// >> i/o operations here <<
			outFileacico<<nrepeat<<' '<<xt[0]<<' '<<xt[1]<<' '<<ut[0]<<' '<<k[0]<<' '<<k[1]<<' '<<td_error<<' '<<rt<<' '<<Vt<<' '<<exp_output[0]<<' '<<output_grad[0]<<' '<<nbasis<<' '<<output_elig[0]<<' '<<ntrial<<' '<<sig[0]<<' '<<sig_out[0]<<endl;


			iter++;
			resetLearning_RL++;

		}

		else // Learning stop!!
		{
			//End learning process
		}

		if(!combinecontrol) // if both systems are steering, y_[] will be set to some superposition of y_ac and y_ico later
		{
			y_[0] = y_ac[0]; // left front wheel
			y_[1] =	y_ac[1]; // right front wheel
			y_[2] =	y_ac[2]; // left rear wheel
			y_[3] =	y_ac[3]; // right rear wheel

		}

	} //here ends AC-control

	// ico learning --------------------------------------------------------------------//
	if(ico_control)
	{
		switch (scenario_flag) {

				case 1:
				{
					//Green
					xt[_ias0] =xt_ico_lowpass2;// input to controller Set sensory signal of robot (State) to actor input xt

					//Blue
					xt[_ias1] =xt_ico_lowpass3;

					xt[_ias2] = irl_lowpass*1.5 ;

					xt[_ias3] = irr_lowpass*1.5 ;

					break;
				}

				case 2:
				{
					/* partial observable case */
					xt[_ias2] = irl_lowpass*1.5;

								xt[_ias3] = irr_lowpass*1.5;


								if (xt_ico2[_ias1] <= 0.8 || xt_ico3[_ias1] <=0.8/*&& xt_ico2[_ias1] > 0.5*/)
									{xt[_ias0] = xt_ico_lowpass2; //0.0 ;// input to controller Set sensory signal of robot (State) to actor input xt

									}
								else
									{xt[_ias0] = 0.0 ;//xt_ico_lowpass2;
									}
								//Blue
								if(xt_ico3[_ias1] <=0.8 || xt_ico2[_ias1] <= 0.8/*&& xt_ico3[_ias1] > 0.5*/ )
									xt[_ias1] = xt_ico_lowpass3; //0.0;

								else
									xt[_ias1] = 0.0; //xt_ico_lowpass3;

								break;


//					if (xt_ico2[_ias1] < 0.6  )
//									{xt[_ias0] =xt_ico_lowpass2;// input to controller Set sensory signal of robot (State) to actor input xt
//
//									}
//								else
//									{xt[_ias0] = 0.0;
//									}
//								//Blue
//								if(xt_ico3[_ias1] < 0.6)
//									xt[_ias1] =xt_ico_lowpass3;
//
//								else
//									xt[_ias1] = 0.0;
//
//								break;
//
//					if (xt_ico2[_ias1] < 0.6)
//						xt[_ias0] =xt_ico_lowpass2;// input to controller Set sensory signal of robot (State) to actor input xt
//
//					else
//						xt[_ias0] = 0.0;
//
//					//Blue
//					if(xt_ico3[_ias1] < 0.6)
//						xt[_ias1] =xt_ico_lowpass3;
//
//					else
//						xt[_ias1] = 0.0;
//
//					break;

				}

				}
		 printf("xt[_ias0]= %f \n xt[_ias1]= %f \n", xt[_ias0],xt[_ias1]);

		//input_angle_s.at(0) = Goal 1 red  [-1(-2),...,1(2)]
		//input_angle_s.at(1) = Goal 2 green [-1(-2),...,1(2)]
		//input_angle_s.at(2) = Goal 3 blue [-1(-2),...,1(2)]
		//input_angle_s.at(3) = Goal 4 yellow [-1(-2),...,1(2)]

		//DISTANCE TO GOALS
		//input_distance_s  = normalized distance from Goal 1 [0,...,1]
		//input_distance_s2 = normalized distance from Goal 2 [0,...,1]
		//input_distance_s3 = normalized distance from Goal 3 [0,...,1]
		//input_distance_s4 = normalized distance from Goal 4 [0,...,1]

		//2) Exploration noise
		 if(!combinecontrol){
		double lp_gain =  0.99;//0.6;
		double sig_nolearning = 5.0;
		//Exploration gradient output
		for(int i =0; i< UDIM /*2*/; i++)
		{
			exploration_g[i] = gauss();
			exploration_lowpass_old_g[i] = exploration_lowpass_g[i];
			exploration_lowpass_g[i] = exploration_lowpass_old_g[i]*lp_gain+(1-lp_gain)*exploration_g[i];
		}

		for(int i =0; i< UDIM /*2*/; i++)
		{
			exp_output[i]= sig_nolearning*exploration_lowpass_g[i]*(lp_gain);
			//printf("ICO %d \n\n", nrepeat);
		}

		printf("\n inside loop ************************");
		 }

		//Check state
		failure_flag_ico = 0;
		if(xt_ico[_ias1] < 0.03 || xt_ico2[_ias1] < 0.03 || xt_ico3[_ias1] < 0.03 || xt_ico4[_ias1] < 0.03 || irl_lowpass*1.5 > 0.2 || irr_lowpass*1.5 > 0.2)
		{
			failure_flag_ico = 1;

		}


		//----------Learning one output

		if(use_Learned_weightsv2) 	//for When finish learning!!//
		{
			k_ico[1] = 0.000307;
			k_ico[2] = 0.000793;
		}

		if(!combinecontrol)
		{
			//u_ico_in[0] = k_ico[0]*xt_ico_lowpass+xt_reflex_angle*5;//+exp_output[0];
			u_ico_in[1] = k_ico[1]*/*xt_ico_lowpass2*/xt[_ias0]+xt_reflex_angle2;//*5;//+exp_output[0]; // Green
			u_ico_in[2] = k_ico[2]*/*xt_ico_lowpass3*/xt[_ias1]+xt_reflex_angle3;//*5;//+exp_output[0]; // Blue
			//u_ico_in[3] = k_ico[3]*xt_ico_lowpass4+xt_reflex_angle4*5;//+exp_output[0];

		}

		if(combinecontrol)
		{

			if(exp_output_decay_ICO==0.0)
			{
				//u_ico_in[0] = k_ico[0]*xt_ico_lowpass+xt_reflex_angle*5;//+exp_output[0];
				u_ico_in[1] = k_ico[1]*/*xt_ico_lowpass2*/xt[_ias0];//+xt_reflex_angle2;//*5;//+exp_output[0];
				u_ico_in[2] = k_ico[2]*/*xt_ico_lowpass3*/xt[_ias1];//+xt_reflex_angle3;//*5;//+exp_output[0];
				//u_ico_in[3] = k_ico[3]*xt_ico_lowpass4+xt_reflex_angle4*5;//+exp_output[0];
			}
			else
			{

				//u_ico_in[0] = k_ico[0]*xt_ico_lowpass+xt_reflex_angle*5;//+exp_output[0];
				u_ico_in[1] = k_ico[1]*/*xt_ico_lowpass2*/xt[_ias0]+xt_reflex_angle2;//*5;//+exp_output[0];
				u_ico_in[2] = k_ico[2]*/*xt_ico_lowpass3*/xt[_ias1]+xt_reflex_angle3;//*5;//+exp_output[0];
				//u_ico_in[3] = k_ico[3]*xt_ico_lowpass4+xt_reflex_angle4*5;//+exp_output[0];


			}
		}

		//Due to simulation error
		/*if(xt_ico2[_ias1]<0.03)
			deri_xt_reflex_angle2 = 0.0;
		if(xt_ico3[_ias1]<0.03)
			deri_xt_reflex_angle3 = 0.0;*/


		////std::cout<<"B derireflex2:" <<deri_xt_reflex_angle2<<"\n"<< "derireflex3:" <<deri_xt_reflex_angle3 <<"\n"<<std::endl;

		if(abs(deri_xt_reflex_angle2)<0.53)//0.55)//0.17) // small update!! thus reduce threshold
		{
			deri_xt_reflex_angle2 = 0.0;
		}

		if(abs(deri_xt_reflex_angle3)<0.53)//0.55)//0.17)
		{
			deri_xt_reflex_angle3 = 0.0;
		}


		//No learning after reset for 500 steps!!! protecting weight to drop!!!
		if(resetLearning_RL<time_to_test_learned_policy)
		{
			deri_xt_reflex_angle2 = 0.0;
			deri_xt_reflex_angle3 = 0.0;
		}

		////std::cout<<"AF derireflex2:" <<deri_xt_reflex_angle2<<"\n"<< "derireflex3:" <<deri_xt_reflex_angle3 <<"\n"<<std::endl;

		//printf("k_ico[0]_red = %f \n k_ico[1]_green = %f \n [2]_blue=%f \n deri_xt_reflex_angle2: %f \n deri_xt_reflex_angle3: %f\n", k_ico[0], k_ico[1], k_ico[2], deri_xt_reflex_angle2, deri_xt_reflex_angle3);


		if(!combinecontrol)
		{
			//k_ico[0] += rate_ico*deri_xt_reflex_angle*xt_ico_lowpass;
			k_ico[1] += rate_ico*abs(deri_xt_reflex_angle2)*abs(/*xt_ico_lowpass2*/xt[_ias0]);
			k_ico[2] += rate_ico*abs(deri_xt_reflex_angle3)*abs(/*xt_ico_lowpass3*/xt[_ias1]);
			//k_ico[3] += rate_ico*deri_xt_reflex_angle4*xt_ico_lowpass4;
		}

		if(combinecontrol)
		{

			//			if(exp_output_decay_ICO==0.0)
			//			{
			//				k_ico[1] += 0;
			//				k_ico[2] += 0;
			//			}
			//			else
			//			{
			//
			//				//k_ico[0] += rate_ico*deri_xt_reflex_angle*xt_ico_lowpass;
			//				k_ico[1] += rate_ico*deri_xt_reflex_angle2*xt_ico_lowpass2;
			//				k_ico[2] += rate_ico*deri_xt_reflex_angle3*xt_ico_lowpass3;
			//				//k_ico[3] += rate_ico*deri_xt_reflex_angle4*xt_ico_lowpass4;
			//
			//			}

			//k_ico[0] += rate_ico*deri_xt_reflex_angle*xt_ico_lowpass;
			k_ico[1] += rate_ico*abs(deri_xt_reflex_angle2)*abs(/*xt_ico_lowpass2*/xt[_ias0]);
			k_ico[2] += rate_ico*abs(deri_xt_reflex_angle3)*abs(/*xt_ico_lowpass3*/xt[_ias0]);
			//k_ico[3] += rate_ico*deri_xt_reflex_angle4*xt_ico_lowpass4;


		}

		if(!combinecontrol)
		{
			//IF REACH GOAL
			if(/*xt_ico[_ias1] < 0.03 ||*/ xt_ico2[_ias1] < 0.03 || xt_ico3[_ias1] < 0.03 || xt_ico4[_ias1] < 0.03)
				reduce_noise_ICO++;

			exp_output[0] = exp_output[0]*exp(-reduce_noise_ICO/500); // 50 decay if reduce_noise > 350 noise gone
			if(abs(exp_output[0])< 0.0001)
				exp_output[0] = 0.0;

			printf("exp_output[0] in !combine = %f : reduce_noise_ICO = %d\n", exp_output[0], reduce_noise_ICO);


		}


		if(combinecontrol)
		{
			//IF REACH GOAL
			if(/*xt_ico[_ias1] < 0.03 ||*/ xt_ico2[_ias1] < 0.05 /*|| xt_ico3[_ias1] < 0.05 || xt_ico4[_ias1] < 0.03*/)
				reduce_noise_ICO++;

			exp_output_decay_ICO = exp(-reduce_noise_ICO/200);//100);
			if(abs(exp_output_decay_ICO)< 0.0001)
				exp_output_decay_ICO = 0.0;

			printf("\n exp_output in combined loop = %f",exp_output[0]);

			u_ico[0] = 1.0*u_ico_in[1]+1.0*u_ico_in[2];


			// Learning one output
			y_ico[0] = scale*1+u_ico[0]; // left front wheel
			y_ico[1] = scale*1-u_ico[0]; // right front wheel
			y_ico[2] = scale*1+u_ico[0]; // left rear wheel
			y_ico[3] = scale*1-u_ico[0]; // right rear wheel

		}


		if(!combinecontrol)
		{

			u_ico[0] = 1.0*u_ico_in[1]+1.0*u_ico_in[2]+exp_output[0];

			// Learning one output
			y_ico[0] = scale*1+u_ico[0]; // left front wheel
			y_ico[1] = scale*1-u_ico[0]; // right front wheel
			y_ico[2] = scale*1+u_ico[0]; // left rear wheel
			y_ico[3] = scale*1-u_ico[0]; // right rear wheel


			y_[0] = y_ico[0]; // left front wheel
			y_[1] =	y_ico[1]; // right front wheel
			y_[2] =	y_ico[2]; // left rear wheel
			y_[3] =	y_ico[3]; // right rear wheel
		}

		outFileicolearning<< nrepeat<<' '<<exp_output[0]<<' '<<u_ico_in[1]<<' '<<u_ico_in[2]<<' '<<u_ico[0]*0.5<<' '<<xt_ico2[_ias0]<<' '<<xt_ico_lowpass2<<' '<<xt_ico3[_ias0]<<' '<<xt_ico_lowpass3<<' '<<xt_reflex_angle2<<' '<<deri_xt_reflex_angle2<<' '<<xt_reflex_angle3<<' '<<deri_xt_reflex_angle3<<' '<<k_ico[1]<<' '<<k_ico[2]<<endl;


		resetLearning_RL++;
	}

	// manual steering
	if(manual_control)
	{

		////std::cout<<"u = forward"<<" \n"<<"j = back"<<"\n"<<"b = TL "<<"\n"<<"k = TR"<<"\n"<<"space = stop \n"<<std::endl;
		for (int i=0;i<4;i++)
			y_[i]=mc[i];

	}

	if(combinecontrol)
	{

		u_ico_lowpass = u_ico_old*0.9 + u_ico[0]*w_ico*0.1;
		ut_lowpass = ut_old*0.9 + ut[0]*w_ac*0.1;

		 // u_ico_lowpass = u_ico_lowpass*0.9 + u_ico[0]*0.1;
		// ut_lowpass = ut_lowpass*0.9 + ut[0]*0.1;

        printf("\n EXP_crazy_output[0] = %g EXP_keep_value float = %f",exp_output[0], EXP_keep_value);

	//	if(!(exp_output[0]<0.0001)){ //learn_combined_weights = false;  // stop learning combined  weights once the robot converges to the correct goal

		if(learn_combined_weights == true){
			//if(rt>0)
			 w_ico = w_ico + rate*(rt*abs(u_ico[0]-u_ico_lowpass))*ut[0];
		//	  w_ico = w_ico + rate*(rt*u_ico[0]) + rate_ss*abs(u_ico_lowpass-u_ico[0])*pow(w_ico,2); //pow(w_ico,2);

			  w_ac =  w_ac + rate*(rt*abs(ut[0]-ut_lowpass))*u_ico[0];

// 			  w_ac = w_ac + rate*(rt*ut[0]) + rate_ss*abs(ut_lowpass-ut[0])*pow(w_ac,2); //pow(w_ac,2);

			//Synaptic normalization

		double w_ico_old = w_ico / (w_ico + w_ac);
			double w_ac_old = w_ac/ (w_ico+ w_ac);

			w_ico = w_ico_old;
			w_ac = w_ac_old;


		}
	//	}


		y_[0] = (w_ico)*y_ico[0]+(w_ac)*y_ac[0]; // left front wheel
		y_[1] =	(w_ico)*y_ico[1]+(w_ac)*y_ac[1]; // right front wheel
		y_[2] =	(w_ico)*y_ico[2]+(w_ac)*y_ac[2]; // left rear wheel
		y_[3] =	(w_ico)*y_ico[3]+(w_ac)*y_ac[3]; // right rear wheel

		/*
    y_[0] = 0.5*y_ico[0]+0.5*y_ac[0]; // left front wheel
    y_[1] =	0.5*y_ico[1]+0.5*y_ac[1]; // right front wheel
    y_[2] =	0.5*y_ico[2]+0.5*y_ac[2]; // left rear wheel
    y_[3] =	0.5*y_ico[3]+0.5*y_ac[3]; // right rear wheel*/
	}
	//added:
	outFileCalibrate << stepnumber_t << "  "<< xt_ico_lowpass2 <<" "<< input_distance_s2 <<" "<< xt_ico_lowpass3 <<" "<< input_distance_s3 << "\n";

	stepnumber_t++;
}

void ACICOControllerV14::stepNoLearning(const sensor* x_, int number_sensors,
		motor* y_, int number_motors)
{

}
double ACICOControllerV14::sigmoid(double num)
{
	return 1./(1.+exp(-num));
}

double ACICOControllerV14::tanh(double num)
{
	return 2./(1.+exp(-2*num))-1.;
}

//----AC network parameters-----------------------------------------------------------------//
/*-------------------------------ACTOR COMPONENTS------------------------------------------*/

/*************************************************************
 *	ACTOR (1)
 *************************************************************/

void ACICOControllerV14::output_policy(double *x /*in*/, double *u /*return*/)
{
	bool lowpassnoise = true;

	double y;
	double V1, V0;
	double max_value, min_value;
	double sig_nolearning;
	double sig0_nolearning;
	double online_error;

	double u_tmp[UDIM /*2*/];
	double u_tmp_non[UDIM /*2*/];

	for(int i =0; i< UDIM /*2*/; i++)
	{
		u_tmp[i] = 0.0;
		u_tmp_non[i] = 0.0;
	}



	//******************

	//-----------ESN nonlinear actor--uncomment this for nonlinear actor-------------------------//
	ut0_lowpass = old_ut*0.8+ ut[0]*0.2;

    EAinput[0] = xt[_ias0];
    EAinput[1] = xt[_ias1];
    EAinput[2] = xt[_ias2]; //irl_lowpass ;//old_ut;
    EAinput[3] = xt[_ias3] ; //irr_lowpass; //ut[0];

    EATrainOutput[0] = u[0] ;//1.0;

	ESN_actor->setInput(ESinput, 4 /*3*/);// sizeof(xt)/*+1*/


    learn_actor = true;
		//if (rt!=0) learn_actor=true;

					//if(exp_output[0]< 0.0001) learn_actor = false;

				//	else
					//	learn_actor = true;
//
////                if (td_error == 0) {online_error = output_grad[0]; }
//  //              else

//0.0033 old stable rate

				ESN_actor->takeStep(ESTrainOutput, 0.9/*0.0035*//*0.0033*//*1.5*/, td_error, learn_actor, output_grad[0]/*ut0_lowpass, param4*/);
//
               //ESN_actor->printMatrix(ESN_actor->endweights); // print weights
//
				//    std::cout<< "output: " << ESN->outputs->val(0, 0) <<std::endl;
//


				u_tmp_non[0] = ESN_actor->outputs->val(0, 0); // Return output from ESN
			//u_tmp[0] = ESN_actor->outputs->val(0, 0); // Return output from ESN
//////


//			std::cout<<"/n********************************u_tmp[0] = "<<u_tmp[0]<<std::endl;

	//***********************

	//-----------ESN nonlinear actor--comment this for nonlinear actor-------------------------//


	u_tmp[0] = k[0]*x[0]+k[1]*x[1] + k[2]*xt[_ias2] + k[3]*xt[_ias3];
	if(nrepeat > 30)
	  u_tmp[0] = u_tmp_non[0];



	//std::cout<<"x[0] = "<<x[0];
//			u_tmp[0] = k[0]*xt[_ias0] + k[1]*xt[_ias1];

//	std::cout<<"********************************u_tmp[0] = "<<u_tmp[0];

	exploration_output(sig /*2D*/, si /*10*/, x /*2D*/);

	//--Lowpass noise-----
	if(lowpassnoise)
	{
		double lp_gain =  0.99;//0.6;
		//Exploration gradient output
		for(int i =0; i< UDIM /*2*/; i++)
		{value_function(xt/*inputs*/); //Replace
		exploration_g[i] = gauss();
		exploration_lowpass_old_g[i] = exploration_lowpass_g[i];
		exploration_lowpass_g[i] = exploration_lowpass_old_g[i]*lp_gain+(1-lp_gain)*exploration_g[i];
		}
		//--------------------
		//--------------------
		if(no_learning_noise)
		{
			V1 = 50;//50.0; //50.0;//1.0;//50.0;//50.0;//1.0;//50.0;//50.0;//50.0;//70.0;//50.0;//1.0;//50.0;//50.0;//1.0; // Max expected value//---------------------------------**** Change
			V0 = -50; //-50.0 ; //0.0 ;//-50.0 ;//0.00; //-50 ;//-3.5;//-50.0; //0.0; //-50.0; //-1.0;//0.0;//0;//0.0;//-125;//0.5; // Min expected value//---------------------------------****  Change
			sig0_nolearning = 5.0 ;//5.0 ; //5.0 ;//5.0;//5.0;//si;

			y = (V1-Vt)/(V1-V0);
			max_value = max(0, y);
			min_value = min(1,max_value); //changing from 1 to 0.1 (0.1,0.33)

			//   if(min_value ==0) min_value = 0.1;


			sig_nolearning = sig0_nolearning*min_value;

			if(abs(sig_nolearning)<th_exp_td_vt)
				sig_nolearning = 0.0;

			for(int i =0; i< UDIM /*2*/; i++)
			{
				exp_output[i]= exp_output_decay*sig_nolearning*exploration_lowpass_g[i]*(lp_gain);
			}

			if((exp_output[0] == 0.0)) learn_combined_weights = false;
			else
				learn_combined_weights = true;

			if((exp_output[0] == 0.0)) exploreEnd = true;

		//	if (exp_output[0]<0.0001) std::cout<<time(NULL)-start_time<<std::endl;

			EXP_keep_value = exp_output[0];
			printf("\n EXP Keep_value in noise calc = %f",EXP_keep_value);

			//printf("no learning low pass noise:  max_value= %f, min_value= %f, exp_output[0]= %f Vt= %f\n", max_value, min_value, exp_output[0], Vt);

		}
		else
		{
			for(int i =0; i< UDIM /*2*/; i++)
			{
				//Morimoto
			//	exp_output[i]=sig[i]*exploration_lowpass_g[i]*(lp_gain);

				//Kimura
				exp_output[i]= exploration_lowpass_g[i]*(lp_gain);
			}

			//printf("learning noise low pass \n");
		}

	}
	else
	{
		if(no_learning_noise)
		{

			V1 = 12.0;  //0;// Max expected value
			V0 = 0; // Min expected value
			sig0_nolearning = si;

			y = (V1-Vt)/(V1-V0);
			max_value = max(0, y);
			min_value = min(1,max_value);
			sig_nolearning = sig0_nolearning*min_value;

			if(abs(sig_nolearning)<th_exp_td_vt)
				sig_nolearning = 0.0;

			for(int i =0; i< UDIM /*2*/; i++)
			{
				exp_output[i]= exp_output_decay*sig_nolearning*gauss();
			}

			//printf("no learning noise:  max_value= %f, min_value= %f, exp_output[0]= %f Vt= %f\n", max_value, min_value, exp_output[0], Vt);

		}
		else
		{
			for(int i =0; i< UDIM /*2*/; i++)
			{
				//Morimoto
				//exp_output[i]=sig[i]*gauss();

				//Kimura
				exp_output[i]= gauss();
			}

			//printf("learning noise \n");
		}

	}

	if(use_Learned_weightsv2) 	//for When finish learning!!//
	{

		//Controller 4  work Well Best

		k[0]= 0.201170;
		k[1]= 0.042301;

		u[0] = k[0]*x[0]+k[1]*x[1];
	}
	else
	{

		for(int i =0; i< UDIM /*2*/; i++)
		{
            printf("\n exp_output in AC loop = %f",exp_output[1]);

            /*u[i]*/u[i] = u_tmp[i] + exp_output[i];
		}

	}

}

/*************************************************************
 *	EXPLORATION (1.1)
 *************************************************************/
void ACICOControllerV14::exploration_output(double *sig /*return*/, double si, double *x /*in*/)
{

	for(int i=0;i<UDIM;i++)
	{
		sig[i] = si/(1+exp(-sig_out[i]));
		//printf("sig [%d] = %f si = %f sig_out [%d] = %f \n", i, sig[i],si, i, sig_out[i]);
	}

}



/*************************************************************
 *	Gaussian random variable: just a sum of uniform distribution
 *	Average = 0, Variance = 1, (1.2)
 *************************************************************/
double ACICOControllerV14::gauss()
{
	double	sum;
	int 	i;

	for( sum = i = 0; i < 12; i++) sum += (1.0*(double)rand()/(RAND_MAX));
	return(sum - 6.0);
}

/*************************************************************
 *	CALCULATE REWARD (2)
 *************************************************************/
double ACICOControllerV14::reward_function(double *x /*state*/, double *u /*action*/)
{

	r_total = 0.0;


	//xt_ico2[_ias1], xt_ico3[_ias1]

// if(exploreEnd == true)
// {
//	//if(xt[_ias1]< range_reflex/*Blue*/)
//	if(xt_ico2[_ias1]< range_reflex/*Green*/)
//		r_total += 1.0; //1.0;
//
//	//else
//		//r_total = 0;
//
//	if(xt_ico3[_ias1]< range_reflex /*|| irl_lowpass > 0.05 || irr_lowpass > 0.05/*Blue*/)
//		r_total -= 1.0 ; //0.0 ;//1.0;
//
// }

 //else
 //{
	 if(xt_ico2[_ias1]< range_reflex/*Green*/)
	 		r_total += 1.0; //1.0;

	 	//else
	 		//r_total = 0;

	 if(xt_ico3[_ias1]< range_reflex /*|| irl_lowpass > 0.05 || irr_lowpass > 0.05/*Blue*/)
	 		r_total -= 1.0 ; //0.0 ;//1.0;
 //}

	if(irl_lowpass > 1.0 /*1.0*/  || irr_lowpass > 1.0 /*1.0*/)
		r_total -= 1.0;

	//r_total +=0.01;
	return r_total;
}

/*************************************************************
 *	CHECK STATE (3)
 *************************************************************/
int ACICOControllerV14::check_limit(double *x)//, double irr_back, double irl_back)
{

	int flag=0;



//	if( xt_ico2[_ias1] < 0.03 || xt_ico3[_ias1] < 0.03 || irl_lowpass*1.5 > 0.2 || irr_lowpass*1.5 > 0.2)
//	{
//		flag = 1;
//
//	}

	if( xt_ico2[_ias1] < 0.03 || xt_ico3[_ias1] < 0.03 || irl_lowpass > 1.5 || irr_lowpass > 1.5)
		{
			flag = 1;

		}




	return flag;


}



/*-------------------------------CRITIC COMPONENTS------------------------------------------*/
/*************************************************************
 *	Initial RBF network (0)
 *************************************************************/
void ACICOControllerV14::init_funcapprox()
{

	//std::cout<<"\n\n"<<std::endl;
	//std::cout<<"----------------------------------------"<<std::endl;
	//std::cout<<"Initial RBF network for AC learning \n"<<std::endl;

	int i;
	int state_index[XDIM /*input, e.g., 4*/];
	double xc[XDIM /*input, e.g.,4*/];

	ngnet->init_incsbox(&VALUE,xdim /*4 input number*/,udim /*1 one output*/);
	ngnet->reset_incsbox(&VALUE);

	//***Set it differently according to a number of your inputs, here only 2 inputs
	basis[_ias0] = 3; // N_IN0_RBF/*3*/; // Number of RBF
	basis[_ias1] = 3; // N_IN1_RBF/*3*/; // Number of RBF
	//	basis[_ias2] = N_IN2_RBF/*3*/; // Number of RBF
	//	basis[_ias3] = N_IN3_RBF/*3*/; // Number of RBF

	//***Set it differently according to a number of your inputs, here only 3 inputs
	//1) Input limitation MAX MIN
	xmax[_ias0] = MAX_IN0;
	xmin[_ias0] = MIN_IN0;

	xmax[_ias1] = MAX_IN1;
	xmin[_ias1] = MIN_IN1;

	//	xmax[_ias2] = MAX_IN2;
	//	xmin[_ias2] = MIN_IN2;
	//
	//	xmax[_ias3] = MAX_IN3;
	//	xmin[_ias3] = MIN_IN3;

	if(print)
	{
		//std::cout<<"Input"<<"xmax["<<_ias0<<"] = "<<xmax[_ias0]<<std::endl;
		//std::cout<<"Input"<<"xmin["<<_ias0<<"] = "<<xmin[_ias0]<<std::endl;
		//std::cout<<"Input"<<"xmax["<<_ias1<<"] = "<<xmax[_ias1]<<std::endl;
		//std::cout<<"Input"<<"xmin["<<_ias1<<"] = "<<xmin[_ias1]<<std::endl;
		//		//std::cout<<"Input"<<"xmax["<<_ias2<<"] = "<<xmax[_ias2]<<std::endl;
		//		//std::cout<<"Input"<<"xmin["<<_ias2<<"] = "<<xmin[_ias2]<<std::endl;
		//		//std::cout<<"Input"<<"xmax["<<_ias3<<"] = "<<xmax[_ias3]<<std::endl;
		//		//std::cout<<"Input"<<"xmin["<<_ias3<<"] = "<<xmin[_ias3]<<std::endl;
		//std::cout<<"\n"<<std::endl;
	}

	//2) find width of each input
	nbasis = 0; // reset
	for(i=0;i<xdim /*4 inputs*/;i++)
	{
		basis_width[i]  = (xmax[i] - xmin[i])/(2.0*basis[i]/*number of centers*/); // variance (the measure of the width of the distribution)
		wbasis[i] = 1.0/basis_width[i]; //width of each input

		if(print)
		{
			//std::cout<<"Varience of each input of RBF net"<<"wbasis["<<i<<"] = "<<basis_width[i]<<" basis["<<i<<"]= "<<basis[i]<<std::endl;
		}
	}

	//3) calculate center of neuron (,e.g. 3*3 = 9 neurons)

	//--------- one input-----------------------//
	//		for(state_index[_ias0]=0; state_index[_ias0]<basis[_ias0] /*3*/; state_index[_ias0]++) // angle left
	//		{
	//			if(print)
	//			{
	//				//std::cout<<"\n"<<std::endl;
	//				//std::cout<<"neuron \t"<<nbasis<<std::endl;
	//			}
	//
	//			for(i=0;i<xdim /*2 inputs*/;i++)
	//			{
	//				xc[i]  = (xmin[i]+basis_width[i])+state_index[i]*2.0*basis_width[i];
	//
	//				if(print)
	//				{
	//					//std::cout<<"Coordinate of center of inputs-> xc["<<i<<"] = "<<xc[i]<< "state_index["<<i<<"] = "<<state_index[i]<<std::endl;
	//				}
	//
	//			}
	//			ngnet->put_incsbox(&VALUE, xdim /*2 inputs*/, 1, xc, wbasis/*1/Variance*/, &nbasis);
	//		}

	//--------- two inputs-----------------------//
	for(state_index[_ias0]=0; state_index[_ias0]<basis[_ias0] /*3*/; state_index[_ias0]++) // angle left
	{
		for(state_index[_ias1]=0; state_index[_ias1]<basis[_ias1] /*3*/; state_index[_ias1]++) // angle right
		{
			if(print)
			{
				//std::cout<<"\n"<<std::endl;
				//std::cout<<"neuron \t"<<nbasis<<std::endl;
			}

			for(i=0;i<xdim /*2 inputs*/;i++)
			{
				xc[i]  = (xmin[i]+basis_width[i])+state_index[i]*2.0*basis_width[i];

				if(print)
				{
					//std::cout<<"Coordinate of center of inputs-> xc["<<i<<"] = "<<xc[i]<< "state_index["<<i<<"] = "<<state_index[i]<<std::endl;
				}

			}
			ngnet->put_incsbox(&VALUE, xdim /*2 inputs*/, 1, xc, wbasis/*1/Variance*/, &nbasis);
		}

	}




	//--------- three inputs-----------------------//
	//
	//	for(state_index[_ias0]=0; state_index[_ias0]<basis[_ias0] /*3*/; state_index[_ias0]++) // angle
	//	{
	//		for(state_index[_ias1]=0; state_index[_ias1]<basis[_ias1] /*3*/; state_index[_ias1]++) // distance
	//		{
	//			for(state_index[_ias2]=0; state_index[_ias2]<basis[_ias2] /*3*/; state_index[_ias2]++) // IR
	//			{
	//				if(print)
	//				{
	//					//std::cout<<"\n"<<std::endl;
	//					//std::cout<<"neuron \t"<<nbasis<<std::endl;
	//				}
	//
	//				for(i=0;i<xdim /*3 inputs*/;i++)
	//				{
	//					xc[i]  = (xmin[i]+basis_width[i])+state_index[i]*2.0*basis_width[i];
	//
	//					if(print)
	//					{
	//						//std::cout<<"Coordinate of center of inputs-> xc["<<i<<"] = "<<xc[i]<< "state_index["<<i<<"] = "<<state_index[i]<<std::endl;
	//					}
	//
	//				}
	//				ngnet->put_incsbox(&VALUE, xdim /*3 inputs*/, 1, xc, wbasis/*1/Variance*/, &nbasis);
	//			}
	//		}
	//
	//	}


	//	//--------- four inputs-----------------------//
	//
	//	for(state_index[_ias0]=0; state_index[_ias0]<basis[_ias0] /*3*/; state_index[_ias0]++) // angle
	//	{
	//		for(state_index[_ias1]=0; state_index[_ias1]<basis[_ias1] /*3*/; state_index[_ias1]++) // distance
	//		{
	//			for(state_index[_ias2]=0; state_index[_ias2]<basis[_ias2] /*3*/; state_index[_ias2]++) // IRL
	//			{
	//				for(state_index[_ias3]=0; state_index[_ias3]<basis[_ias3] /*3*/; state_index[_ias3]++) // IRR
	//				{
	//					if(print)
	//					{
	//						//std::cout<<"\n"<<std::endl;
	//						//std::cout<<"neuron \t"<<nbasis<<std::endl;
	//					}
	//
	//					for(i=0;i<xdim /*4 inputs*/;i++)
	//					{
	//						xc[i]  = (xmin[i]+basis_width[i])+state_index[i]*2.0*basis_width[i];
	//
	//						if(print)
	//						{
	//							//std::cout<<"Coordinate of center of inputs-> xc["<<i<<"] = "<<xc[i]<< "state_index["<<i<<"] = "<<state_index[i]<<std::endl;
	//						}
	//
	//					}
	//					ngnet->put_incsbox(&VALUE, xdim /*4 inputs*/, 1, xc, wbasis/*1/Variance*/, &nbasis);
	//				}
	//			}
	//		}
	//
	//	}

	//std::cout<<"\n"<<std::endl;
	//std::cout<<"Complete Initial RBF network for AC learning : ) "<<std::endl;
	//std::cout<<"---------------------------------------- \n"<<std::endl;

}

/*************************************************************
 *	Value function calculation (4)
 *************************************************************/
double ACICOControllerV14::value_function(double *x /*in*/)
{
	double Value;

	ngnet->incsbox_output(&VALUE, x, &Value, &nbasis);

	return Value;
}

/*************************************************************
 *	EXPLORATION GRADIENT METHOD (5)
 *************************************************************/
void ACICOControllerV14:: exploration_grad(double *exp_output, double *sig_elig, double *sig, double si)
{
	int i;


	for(i=0;i<UDIM /*2 dimension*/;i++)
	{
		///Morimoto
		//sig_elig[i] = (exp_output[i]*exp_output[i] /*noise^2*/ - sig[i]*sig[i] /*sig^2*/)*(1 - (sig[i]/si))/(sig[i]*sig[i]);
		//sig_elig[i] = clip(sig_elig[i], -1.0, 1.0);

		///Kimura
		sig_elig[i] = (exp_output[i]*exp_output[i] - sig[i]*sig[i])*(1 - (sig[i]/si));
		sig_elig[i] = clip(sig_elig[i], -1.0, 1.0);
		//printf("sig_elig %f \n\n\n\n", exp_output[i]*exp_output[i] - sig[i]*sig[i]);

	}


}


/*************************************************************
 *	UPDATE VALUE FUNCTION TRACE (6)
 *************************************************************/

void ACICOControllerV14::update_valuefunction_trace(double *x)
{
	ngnet->incsbox_trace(&VALUE, x, lambda_v /*trace, set to 0.0 = no trace*/, &nbasis /*number of hidden neurons*/);
}

/*************************************************************
 *	UPDATE VALUE FUNCTION WEIGHTS (7)
 *************************************************************/

void ACICOControllerV14::update_valuefunction(double *x, double td/*TDerror*/, double rate)
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
	ngnet->incsbox_update(&VALUE, x, td_error_tmp, rate /*e.g., 0.5*/, &nbasis /*number of hidden neurons e.g., initial 27*/, wbasis /*varience of input0, input1*/, error_thresh /*e.g. 0.1*/, unit_activation_thresh /*e.g., 0.6*/);
}

/*************************************************************
 * 	UPDATE POLICY TRACE (8)
 *************************************************************/
void ACICOControllerV14::update_policy_trace(double *xt, double lambda)
{

	// output_elig = noise * input => Dimension of output_elig = WDIM



//**********************************
//	output_elig[0] *= lambda;
//	output_elig[0] += /*output_grad[0]*//*exp_output[0]*/*xt[0]; /*noise0 of output0 x input0, AC-RL*/ // U = XW
////
//	output_elig[1] *= lambda;
//	output_elig[1] += /*output_grad[0]*//*exp_output[1]*/*xt[1]; /*noise0 of output0 x input0, AC-RL*/ // U = XW
//**********************************

	//	output_elig[2] *= lambda;
	//	output_elig[2] += output_grad[0]*xt[1]; /*noise1 of output1 x input1, AC-RL*/ // U = XW
	//
	//	output_elig[3] *= lambda;
	//	output_elig[3] += output_grad[1]*xt[1]; /*noise1 of output1 x input1, AC-RL*/ // U = XW
	//output_elig[3] += output_grad[0]*xt[1]; /*noise1 of output1 x input1, AC-RL*/ // U = XW --------* same

	//	output_elig[4] *= lambda;
	//	output_elig[4] += output_grad[0]*xt[2]; /*noise1 of output1 x input1, AC-RL*/ // U = XW
	//
	//	output_elig[5] *= lambda;
	//	output_elig[5] += output_grad[1]*xt[2]; /*noise1 of output1 x input1, AC-RL*/ // U = XW
	//
	//	output_elig[6] *= lambda;
	//	output_elig[6] += output_grad[0]*xt[3]; /*noise1 of output1 x input1, AC-RL*/ // U = XW
	//
	//	output_elig[7] *= lambda;
	//	output_elig[7] += output_grad[1]*xt[3]; /*noise1 of output1 x input1, AC-RL*/ // U = XW

	//	output_elig[4] = 0.0; /*noise1 of output1 x input1, AC-RL*/ // U = XW
	//	output_elig[5] = 0.0; /*noise1 of output1 x input1, AC-RL*/ // U = XW
	//	output_elig[6] = 0.0; /*noise1 of output1 x input1, AC-RL*/ // U = XW
	//	output_elig[7] = 0.0; /*noise1 of output1 x input1, AC-RL*/ // U = XW

	//	if(xt[2] <= 0.00001 && xt[3] <= 0.00001)
	//	{
	//		output_elig[0] *= lambda;
	//		output_elig[0] += output_grad[0]*xt[0]; /*noise0 of output0 x input0, AC-RL*/ // U = XW
	//
	//		output_elig[1] *= lambda;
	//		output_elig[1] += output_grad[1]*xt[0]; /*noise0 of output1 x input0, AC-RL*/ // U = XW
	//
	//		output_elig[2] *= lambda;
	//		output_elig[2] += output_grad[0]*xt[1]; /*noise1 of output1 x input1, AC-RL*/ // U = XW
	//
	//		output_elig[3] *= lambda;
	//		output_elig[3] += output_grad[1]*xt[1]; /*noise1 of output1 x input1, AC-RL*/ // U = XW
	//		//output_elig[3] += output_grad[0]*xt[1]; /*noise1 of output1 x input1, AC-RL*/ // U = XW --------* same
	//
	//		output_elig[4] = 0.0; /*noise1 of output1 x input1, AC-RL*/ // U = XW
	//		output_elig[5] = 0.0; /*noise1 of output1 x input1, AC-RL*/ // U = XW
	//		output_elig[6] = 0.0; /*noise1 of output1 x input1, AC-RL*/ // U = XW
	//		output_elig[7] = 0.0; /*noise1 of output1 x input1, AC-RL*/ // U = XW
	//
	//	}
	//	else
	//	{
	//
	//		output_elig[0] = 0.0; /*noise1 of output1 x input1, AC-RL*/ // U = XW
	//		output_elig[1] = 0.0; /*noise1 of output1 x input1, AC-RL*/ // U = XW
	//		output_elig[2] = 0.0; /*noise1 of output1 x input1, AC-RL*/ // U = XW
	//		output_elig[3] = 0.0; /*noise1 of output1 x input1, AC-RL*/ // U = XW
	//
	//		output_elig[4] *= lambda;
	//		output_elig[4] += output_grad[0]*xt[2]; /*noise1 of output1 x input1, AC-RL*/ // U = XW
	//
	//		output_elig[5] *= lambda;
	//		output_elig[5] += output_grad[1]*xt[2]; /*noise1 of output1 x input1, AC-RL*/ // U = XW
	//
	//		output_elig[6] *= lambda;
	//		output_elig[6] += output_grad[0]*xt[3]; /*noise1 of output1 x input1, AC-RL*/ // U = XW
	//
	//		output_elig[7] *= lambda;
	//		output_elig[7] += output_grad[1]*xt[3]; /*noise1 of output1 x input1, AC-RL*/ // U = XW
	//	}

}
/*************************************************************
 *	UPDATE POLICY WEIGHTS (9)
 *************************************************************/
void ACICOControllerV14::update_policy(double td_error, double rate)
{
	int i;

	//	for(i=0;i<WDIM /*4 inputs && 2 outputs = 8 weights*/;i++)
	//	{
	//		k[i] += rate*td_error*output_elig[i]; // Actor weights = lr*TDerror*(input*noise)
	//	}

// RLS TD(0) learning

/*if(exp_output[0]>0.0005)
{
	gain0 = (roh0*xt[0])/(rate_rls + roh0*xt[0]*xt[0]);

	roh0 = (1/rate_rls)*(roh0 - gain0*xt[0]*roh0);

	gain1 = (roh1*xt[0])/(rate_rls + roh1*xt[1]*xt[1]);

	roh1 = (1/rate_rls)*(roh1 - gain1*xt[1]*roh1);

	k[0] += gain0*(td_error)*output_grad[0];

	k[1] += gain1*(td_error)*output_grad[0];

}*/
//*************************
	ut0_lowpass = old_ut*0.8+ ut[0]*0.2;
//	ut0_lowpass = old_ut*0.8+ ut[0]*k[0]*0.2;
//	ut1_lowpass = old_ut1*0.8+ ut[0]*k[1]*0.2;
//	ut2_lowpass = old_ut2*0.8+ ut[0]*k[2]*0.2;
//	ut3_lowpass = old_ut3*0.8+ ut[0]*k[3]*0.2;




	//-----------ESN nonlinear actor--comment this for nonlinear actor-------------------------//
	k[0] += rate*(td_error)*/*xt[0]*/xt[_ias0]/*(ut0_lowpass)*/*output_grad[0]; //exp_output[0]; //*output_grad[0]; //*output_elig[0];
	k[1] += rate*(td_error)*/*xt[1]*/xt[_ias1]/*(ut0_lowpass)*/*output_grad[0]; //exp_output[0]; //*output_grad[0]; //output_elig[1];

	k[2] += rate*(td_error)*/*xt[1]*/xt[_ias2]/*(ut0_lowpass)*/*output_grad[0];
	k[3] += rate*(td_error)*/*xt[1]*/xt[_ias3]/*(ut0_lowpass)*/*output_grad[0];




//	if(k[0] && k[1] < 0) k[0]=k[1]=0;
//****************************
	//	k[2] += rate*td_error*output_elig[2];
	//	k[3] += rate*td_error*output_elig[3];
	//
	//	//ICO learning
	//	k[4] += rate_ico*deri_reflex_irl*xt[_ias2];
	//	k[5] -= rate_ico*deri_reflex_irl*xt[_ias2];
	//	k[6] -= rate_ico*deri_reflex_irr*xt[_ias3];
	//	k[7] += rate_ico*deri_reflex_irr*xt[_ias3];


//	sig_out[0] += rate*td_error*sig_elig[0];
//	sig_out[1] += rate*td_error*sig_elig[1];
	//printf("sig_elig[0] = %f : sig_out = %f Dtd_error = %f td_error = %f\n", sig_elig[0],sig_out[0],td_error-td_error_old, td_error);

}

//----AC network parameters-----------------------------------------------------------------//

