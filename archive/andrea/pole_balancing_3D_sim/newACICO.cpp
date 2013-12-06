/*
 * newACICO.cpp
 *
 *  Created on: Dec 28, 2012
 *      Author: andrej
 */
#include "newACICO.h"
#include "ngnet.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>



/*************************************************************
 *	RBF network
 *************************************************************/
void newACICO::init_funcapprox()
{
	int i;
	int state_index[XDIM];
	double xc[XDIM];

	ngnet->init_incsbox(&VALUE, XDIM, 1);
	ngnet->reset_incsbox(&VALUE);
	//

	for (i = 0; i < XDIM; i++)
	{
		basis[i] = 4; // Number of RBF
	}
	//1) Input limitation MAX MIN------------------------------------------------CLEAR
	xmax[_X] = xt_limit; // 2.6 m, KOH 2.4  NOT
	xmin[_X] = -xt_limit; // -2.6 m, KOH -2.4  NOT

	xmax[_V] = 3.5;//26.0; //2.6x10 velocity, KOH 3.5 NOT
	xmin[_V] =-3.5;//-26.0; //-2.6x10 velocity, KOH -3.5 NOT
	for (i = 2; i < XDIM; i+=2)
	{
		xmax[i] = theta_limit; // 12/180*PI = 12 deg =   0.2094 rad EQ
		xmin[i] =-theta_limit; // -12/180*PI = -12 deg = -0.2094 rad EQ

		xmax[i+1] = 2.0;//180.0/180.0*3.14152; //3.1416 rad/s, KOH 2.0 NOT
		xmin[i+1] =-2.0;//-180.0/180.0*3.14152; //-3.1416 rad/s, KOH -2.0 NOT
	}
	//2) find width of each input ------------------------------------------------CLEAR
	nbasis = 0; // reset
	for(i=0;i<XDIM /*4 inputs*/;i++)
	{
		basis_width[i]  = (xmax[i] - xmin[i])/(2.0*basis[i]/*number of centers*/);
		wbasis[i] = 1.0/basis_width[i]; //width of each input
	}
	int basisnumber = 1;
	//3) calculate center of neuron ("basisnumber" neurons)-------------------------------------CLEAR
	/*
	 * This was changed from the previous nested "for"-loop
	 */
	for(int i1 = 0; i1 < XDIM; i1++)
	{
		basisnumber *= basis[i1]; //typically 162 or 256
	}
	for (int i2 = 0; i2 < basisnumber; i2++)
	{
		for (int i1 = 0; i1 < XDIM; i1++)
		{
			state_index[i]++;
			if (state_index[i] == basis[i])
			{
				state_index[i] = 0;
			}
			else
			{
				break;
			}
		}
		for(int i1=0;i1<XDIM /*4 inputs*/;i1++)
		{
			xc[i]  = (xmin[i]+basis_width[i])+state_index[i]*2.0*basis_width[i];
		}
		ngnet->put_incsbox(&VALUE, XDIM, 1, xc, wbasis, &nbasis);
	}
}

/*************************************************************
 *	End RBF networksizeof
 *************************************************************/

/*************************************************************
 *	ICO network
 *************************************************************/
void newACICO::init_icolearning()
{
	printf("activate ico learning \n\n");
	//Input limitation MAX MIN
	xmax[_X] = xt_limit; // 2.6 m, KOH 2.4  NOT
	xmin[_X] = -xt_limit; // -2.6 m, KOH -2.4  NOT

	xmax[_V] = 3.5;//26.0; //2.6x10 velocity, KOH 3.5 NOT
	xmin[_V] =-3.5;//-26.0; //-2.6x10 velocity, KOH -3.5 NOT

	for (i = 2; i < XDIM; i+=2)
	{
		xmax[i] = theta_limit; // 12/180*PI = 12 deg =   0.2094 rad EQ
		xmin[i] =-theta_limit; // -12/180*PI = -12 deg = -0.2094 rad EQ

		xmax[i+1] = 2.0;//180.0/180.0*3.14152; //3.1416 rad/s, KOH 2.0 NOT
		xmin[i+1] =-2.0;//-180.0/180.0*3.14152; //-3.1416 rad/s, KOH -2.0 NOT
	}

}

/*************************************************************
 *	End ICO network
 *************************************************************/

/*************************************************************
 *	Gaussian random variable: just a sum of uniform distribution
 *	Average = 0, Variance = 1
 *************************************************************/

double newACICO::gauss()
{
	double	sum;
	int 	i;

	for( sum = i = 0; i < 12; i++) sum += (1.0*(double)rand()/(RAND_MAX));
	return( sum - 6.0);
}


/*************************************************************
 *	EXPLORATION Gradient
 *************************************************************/

void newACICO::exploration_grad(double *exp_output, double *sig_elig, double *sig, double si)
{
	int i;

	for(i=0;i<UDIM;i++)
	{
		///Morimoto
		sig_elig[i] = (exp_output[i]*exp_output[i] /*noise^2*/ - sig[i]*sig[i] /*sig^2*/ )*(1 - (sig[i]/si))/(sig[i]*sig[i]);
		sig_elig[i] = clip(sig_elig[i], -1.0, 1.0);

		///Kimura
		//sig_elig[i] = (exp_output[i]*exp_output[i] /*noise^2*/ - sig[i]*sig[i] /*sig^2*/ )*(1 - (sig[i]/si)); //e5
		//sig_elig[i] = clip(sig_elig[i], -1.0, 1.0);
	}
}


/*************************************************************
 *	EXPLORATION
 *************************************************************/
void newACICO::exploration_output(double *sig, double si, double *x)
{

	//3)-----------Sensory input limit
	for(int i=0;i<UDIM;i++)
	{

		//Morimoto
		sig[i] = si/(1+exp(-sig_out[i]));

		//Kimura
		//sig[i] = si/(1+exp(-sig_out[i]));
		//printf("sig [0] = %f si = %f sig_out = %f \n", sig[i],si,sig_out[i]);
	}

}




void newACICO::output_policy(double *x, double *u)
{

	int i;
	double u_tmp = 0.0;
	double sig0_nolearning;
	double th_exp_td_vt = 0.05;


	if (learning)
	{
		for(i=0;i<XDIM;i++)
		{
			u_tmp += k[i]*x[i];
		}

		exploration_output(sig, si /*10*/, x);

		//Low pass filer noise NOT USED
		double lp_gain = 0.98;
		exploration_g = gauss();
		exploration_lowpass_old_g = exploration_lowpass_g;
		exploration_lowpass_g = exploration_lowpass_old_g*lp_gain+(1-lp_gain)*exploration_g;




		if(learningNoise) // Morimoto Work best
		{
			if (stopExploration) // Stop exploration
			{if(reset>=stopafter)
				exp_output[0]= 0.0;
			else//if(reset<stopafter)
				exp_output[0]= sig[0]*gauss();//*0.5;//exploration_lowpass_g;
			}
			else
			{
				exp_output[0]= sig[0]*gauss();//exploration_lowpass_g;
			}
		}
		else
		{
			Vmax = 0.0;// Max expected value//---------------------------------**** Change
			Vmin = -20.0;// Min expected value//---------------------------------****  Change
			sig0_nolearning = 1.5;//si;

			y = (Vmax-Vt)/(Vmax-Vmin);
			//y = exp(-0.0069*steps);
			//y = 1-steps/50000;
			max_value = max(0, y);
			min_value = min(1,max_value);
			sig_gain = sig0_nolearning*min_value;

			if(abs(sig_gain)<th_exp_td_vt)
				sig_gain = 0.0;

			exp_output[0]= sig_gain*gauss();//exp_output_decay*sig_nolearning*exploration_lowpass_g[i]*(lp_gain);
		}

		// Output to control cart!
		u[0] = u_tmp+exp_output[0];

	}


	else
	{
		u[0] = 0;
		for (int i = 0; i < XDIM; i++)
			u[0] += k[i]*x[i];
	}

	output = u[0];

}

/*************************************************************
 *	UPDATE ACTOR WEIGHTS
 *************************************************************/
void newACICO::newACICO::update_policy(double td_error, double rate)
{
	int i;

	//Calculate adaptive noise //Morimoto
	sig_out[0] += rate*td_error*sig_elig[0];

	if(learning)
	{
		//Calculate weights
		for(i=0;i<XDIM /*4 inputs = 4 weights*/;i++)
			k[i] += rate*td_error*output_elig[i]; // Actor weights


		//--NOT important ----add koh limit weights
		int threshold_k = 10;

		for(i=0;i<XDIM /*4 inputs = 4 weights*/;i++)
		{
			if(abs(k[i]) > threshold_k)
				k[i]  = 0.0;
		}
	}
	else
	{	//2 time reset
		//	k[0] = 0.655846;
		//	k[1] = 0.935163;
		//	k[2] = 13.996960;
		//	k[3] = 6.401443;

		// Combine 53 trials, -0.5, -9 deg
		//	k[0] = 0.154928;
		//	k[1] = 0.974038;
		//	k[2] = 3.714418;
		//	k[3] = 4.714312;

		// Combine 85 trials, -2.0, -10 deg
		//	k[0] = -1.593497;
		//	k[1] = 0.309366;
		//	k[2] = 2.341801;
		//	k[3] = 2.752788;

		//initialX = 1.000000; initialV = 0.000000; initialTH = 9.999997; initialOM = 0.000000;
		//k[_X] = 0.406465; k[_V] = 1.237400; k[TH] = 4.959975; k[OM] = 5.780908;

		//initialX = 2.000000; initialV = 0.000000; initialTH = 10.999988; initialOM = 0.000000;
		//k[_X] = -7.719890; k[_V] = -1.058514; k[TH] = 8.607580; k[OM] = 11.238556;


		//initialX = -2.4; initialV = 0.000000; initialTH = 7; initialOM = 0.000000;

		//	 k[0] = -8.868151;
		//	 k[1] = 0.071701;
		//	 k[2] = 5.463790;
		//	 k[3] = 17.170846;


		//x_int =  1.400000, v_int = 0.000000, th_int = -0.191986 (-11.000000 deg), om_int = 0.000000

		//k[0] = -0.915862;
		//k[1] = -0.054969;
		//k[2] = 1.451312;
		//k[3] = 2.823070;


		//x_int =  2.400000, v_int = 0.000000, th_int = -0.209440 (-12.000000 deg), om_int = 0.000000
		//Combinatorial learning

		k[0] = -2.154041;
		k[1] = -0.152369;
		k[2] = 8.574895;
		k[3] = 0.883522;
		//K: 1.52, 0.51, 0,44 -0.64
	}
	//	printf("sig_elig[0] = %f : sig_out = %f td_error = %f\n", sig_elig[0],sig_out[0],td_error);
}

/*************************************************************
 *	ETRACE of input x output
 *************************************************************/
void newACICO::update_policy_trace(double *xt, double lambda)
{
	int i;

	for(i=0;i<XDIM;i++)
	{
		output_elig[i] *= lambda;
		//output_elig[i] += output_grad[0]*xt[i]; /*output x input*/
		output_elig[i] -= output_grad[0]*xt[i]; /*output x input*/
	}
}

/*************************************************************
 *	CALCULATE REWARD
 *************************************************************/
double newACICO::reward_function(double *x, double *u)
{
	double r=0.0;
	double bias_xt = xt_limit/6;//0.5;
	double bias_theta = theta_limit/6;//0.0175;
	/*if(abs(x[_X]) < (xt_limit-bias_xt)/3)
		r = 1.0;//+= -1.0;

	if(abs(x[TH]) < (theta_limit-bias_theta)/3)
		r = 1.0;//+= -1.0;*/

	//if(abs(x[TH+2]) < (theta_limit-bias_theta)/3)
	//	r = 1.0;//+= -1.0;




	if(x[_X] < -(xt_limit-bias_xt) /* -2.4 deg*/ || x[_X] > (xt_limit-bias_xt) /* 2.4*/)
		r = -1.0;//+= -1.0;

	for (int i = 2; i < XDIM; i+=2)
	{
		if(x[i] < -(theta_limit-bias_theta) || x[i] > (theta_limit-bias_theta) )
			r = -1.0;//+= -1.0;
	}

	return r;


}

double newACICO::value_function(double *x)
{
	double Value;

	ngnet->incsbox_output(&VALUE, x, &Value, &nbasis);

	//printf("Value %f \n", Value);

	return Value;
}

void newACICO::update_valuefunction( double *x, double td, double rate)
{
	double td_error_tmp[1];
	td_error_tmp[0] = -td;
	ngnet->incsbox_update( &VALUE, x, td_error_tmp, rate, &nbasis, wbasis, error_thresh, unit_activation_thresh);
}

void newACICO::update_valuefunction_trace(double *x)
{
	ngnet->incsbox_trace(&VALUE, x,lambda_v/*No trace if set to 0.0*/, &nbasis);
}

int newACICO::check_limit(double *x)
{
	int flag=0;

	if(x[_X] < -xt_limit || x[_X]  > xt_limit)
	{
		flag = 1;
	}
	for (int i = 2; i < XDIM; i+=2)
	{
		if(x[i] < -theta_limit || x[i] > theta_limit)
		{
			flag = 1;
		}
	}


	return flag;

}


/*************************************************************
 *	RESET INITIAL STATE after fail!!!!
 *************************************************************/


void newACICO::reset_state(double *x)
{
	int i;

	for(i=0;i<XDIM;i++)
		xt[i] = 0.0;

}

void newACICO::step(const sensor* sensors, int sensornumber, motor* motors, int motornumber)
{
	//WARNING überprüfen


	xt[0] = sensors[0] * 0.32;
	xt[1] = (xt[0] - xt_prev[0])*100;

	xt[2] = sensors[4];
	xt[3] = (xt[2]-xt_prev[2])*10;
	xt_prev[0] = xt[0];
	xt_prev[2] = xt[2];

	if (XDIM > 4)
	{
		xt[4] = sensors[5];
		xt[5] = xt[4]-xt_prev[4];
		xt_prev[4] = xt[4];
	}

	for (int i1 = 0; i1 < 4; i1++)
	{
		if (abs(k[i])>40) k[i1] = 0;
		if (abs(kico[i])>40) kico[i1] = 0;
	}


	/*
	 * Failure Handling
	 */
	if(failure_flag)
	{
		ntrial++;
		motors[0] = gauss();
		xt[0] = 0;
		xt[1] = 0;
		xt[2] = 0;
		xt[3] = 0;

		/*if(iter >= MAX_ITER/2)
				reset++;
			else
				ntrial++;

			if(random_state_set)
			{
				reset_state(xt);

				initialX = xt[_X];
				initialV = xt[_V];
				initialTH = xt[TH];
				initialOM = xt[OM];
			}
			else
			{

				initialV = 0.0001*uniform_dist();
				initialOM = 0.0001*uniform_dist();

				xt[_X] = initialX;
				xt[_V] = initialV;
				xt[TH] = initialTH;
				xt[OM] = initialOM;

			}*/


		// RESET values

		Vt_old = 0.0;
		//ntrial++;
		//[TODO]print stuff here
		simulations_run++;
		/*std::cout << "steps ran: " << iter << std::endl;
			std::cout << "Trial Nr: " << simulations_run << std::endl;*/
		//usleep(300000);
		iter = 0;

		steps = 0;

		rt_sum = 0;
		//Save data to analyze//
		acum_reward = 0.0; // RESET acc reward
		//std::cout << "reward reset" << std::endl;
		//balance_time_log[ACRLntrial] = balance_time;
		//sig_out_log[ntrial] = sig_out[0];
		balance_time = 0.0;
		//printf("number of trial %d \n", ntrial);
		exploration_grad(exp_output, sig_elig, sig, si);
		for(i=0;i<UDIM;i++)
		{
			if(learningNoise)
				//Morimoto
				output_grad[i] = clip(exp_output[i]/(sig[i]*sig[i]),-1.0, 1.0); // with noise learning
			else		//Doya & no learning noise				Vt_old = gam*Vt;
				output_grad[i] = clip(exp_output[i],-1.0, 1.0); // with noise learning

			//Kimura
			//output_grad[i] = clip(exp_output[i]/(sig[i]),-1.0, 1.0); // with noise learning
		}
	}

	/*
	 * 	End of Failure Handling
	 */
	/************************DURING LEARNING STATE************************************************/
	if(iter < MAX_ITER && ntrial < MAX_TRIAL && reset < number_of_reset)
	{
		if (combination == 0)
		{
			//-------ICO learning--------------//
			output_icolearning(xt, utico);
			//--------------------------------//
			//ICO learning
			motors[0] = 3*utico[0];
		}
		else if (combination == 1)
		{
			//-Actor critic reinforcement learning-//
			output_policy(xt, ut);
			//--------------------------------//
			//AC learning
			motors[0] = 10*ut[0];
		}
		else
		{
			output_policy(xt, ut);
			output_icolearning(xt, utico);

			motors[0] = ((1-alpha)*ut[0]+alpha*utico[0]);
		}
		/*
		 * Reward
		 */
		rt = reward_function(xt,ut); //return -1 or zero
		rt_sum = gam*rt_sum+rt;//pow(gam,iter)*rt;
		steps++;
		balance_time += dt;

		/* Check state limit */
		/* Value function estimation for current state */
		//replaced by ESN here
		if (!useESN)
		{
			Vt = value_function(xt);
		}
		else
		{
			//Update ESN
			//collect desired output
			float trainingOutput[1];
			trainingOutput[0] = rt_sum/20;
			//colect iputs
			float Inputs[XDIM+1];
			for (int i = 0; i < XDIM; i++)
				Inputs[i] = xt[i];
			Inputs[4] = td_error;
			//take step and collect output
			ESN->setInput(Inputs, XDIM+1);
			ESN->takeStep(trainingOutput, 0.998, td_error, true);
			Vt = ESN->outputs->val(0,0)*20;
		}


		if(failure_flag)
		{
			//Vt = 0;!!
			td_error = 0;
			Vt_old = gam*Vt;
			td_error = clip(td_error, -td_limit /*-1*/, td_limit /*1*/); /// CLIP limit TD error
		}
		else
		{

			td_error = rt + gam*Vt - Vt_old;
			td_error = clip(td_error, -td_limit /*-1*/, td_limit /*1*/); /// CLIP limit TD error

		}

		Vt_old = Vt;
		failure_flag = 0;




		/* Update parameters for value function approximator */
		//update_valuefunction_trace(xt);
		//update_valuefunction(xt, td_error, rate_valuefunction);



		/* Update parameters for policy */
		update_policy_trace(xt, lambda_p);
		update_policy(td_error, rate_policy);



		//AC RL
		p1 = &nbasis;
		total_trials = ntrial + reset;
		iter++;
	}
	else
	{
		reset_simulation();
	}
}

void newACICO::reset_simulation()
{
	/*
	 * WARNING this function kills all learning progress
	 */

	//ESN->endweights.fill(0);

	for (int i1 = 0; i1 < 4; i1++)
	{
		k[i1] = 0;
		kico[i1] = 0;
	}

	ntrial = 0;
	iter = 0;
	reset = 0;
	steps = 0;
	total_trials = 0;
	simulations_runned++;

	if (simulations_runned == 1)
	{
		indexT += sweepStepT;
		if (indexT > finT)
		{
			indexT  = initT;
			indexX +=  sweepStepX;
		}

		if (indexX > finX)
		{
			exit(1);
		}
		else
		{
			simulations_runned = 0;
			std::cout << "\n";

		}
	}

	init_funcapprox();







	nbasis = 0;
	output = 0.0;
	Vt = 0.0;
	reset = 0;
	simulations_run = 0;
	Reflex = 0.0,
			deri_Penalty_actual = 0.0; Penalty_pre = 0.0;
	Penalty = 0.0; deri_Penalty = 0.0;
	for (int i = 0; i < XDIM; i++)
	{
		x_com[i] = 0.0;
	}
	stepcounter = 0;
	xt_limit = 2.37;//2.6;
	theta_limit = 0.2;
	rt_sum = 0;

	acum_reward = 0.0;
	balance_time = 0.0;
	failure_flag = 1; //<--- initialize
	td_error = 0.0;
	save = 1;
	dt = 0.01;
	si = 5; // output factor
	unit_activation_thresh = 0.6;
	error_thresh = 0.1;
	tau = 0.09;
	tau_elig = 0.01;
	tau_elig_output = 0.05;
	kappa    = 0.2;
	td_limit = 1.0;
	rate_valuefunction = 0.5;
	rate_policy = 0.5;//0.5;//0.001;//kimura
	gam = 0.95;
	lambda_v = 0.0;
	lambda_p = 0.0;
	learningRate_ico_value = 0.2;  //0.01;//0.01; 0.001
	learningRate_ico = 0.1;

}
/*************************************************************
 *	ICO learning
 *************************************************************/

void newACICO::output_icolearning(double *x_ico, double *u_ico)
{
	double * maxval = new double[XDIM];

	maxval[_X] = 2.35;
	maxval[_V] = 3;
	for(int i = 2; i < XDIM; i+=2)
	{
		maxval[i] = 0.205;
		maxval[i+1] = 0.1;

	}


	/*double maxtheta = 0.2094384;
	double mintheta = -0.2094384;*/

	;




	//To scale to between +-1



	x_com[_X] = ((x_ico[_X]-maxval[_X])/(maxval[_X]*2))*2-1;

	if(x_com[_X] >1)
		x_com[_X] = 1;
	if(x_com[_X] <-1)
		x_com[_X] = -1;

	x_com[_V] = ((x_ico[_V]-maxval[_V])/(maxval[_V]*2))*2-1;

	if(x_com[_V]>1)
		x_com[_V] = 1;
	if(x_com[_V]<-1)
		x_com[_V] = -1;
	for(int i = 2; i < XDIM; i+=2)
	{
		x_com[i] = ((x_ico[i]-maxval[i])/(maxval[i]*2))*2-1;

		if(x_com[i]>1)
			x_com[i] = 1;
		if(x_com[i]<-1)
			x_com[i] = -1;
		x_com[i+1] =  ((x_ico[i+1]-maxval[i+1])/(maxval[i+1]*2))*2-1;
		if(x_com[i+1]>1)
			x_com[i+1] = 1;
		if(x_com[i+1]<-1)
			x_com[i+1] = -1;
	}



	// Remember the old state
	Penalty_pre =  Penalty;


	//#### (1)
	// Setting reflex
	// Reflex action is trigger
	// when theta > +-0.2007 rad (+-11.5 deg)
	// or when x > +- 2.35 m

	float reflex_treshold = 0.2;
	Reflex = 0.0; // for reflex action
	Penalty = 0.0; // for learning
	if(x_ico[_X] < -2.37 )
	{
		Reflex = 1.0;   // for reflex action
		Penalty = -1.0; // for learning
	}
	if(x_ico[_X] > 2.37 )
	{
		Reflex = -1.0;   // for reflex action
		Penalty = -1.0; // for learning
	}

	for (int i = 2; i < XDIM; i+=2)
	{
		if(x_ico[i] > reflex_treshold)
		{
			Reflex = 1.0;   // for reflex action
			Penalty = -1.0; // for learning
		}
		if(x_ico[i] < -reflex_treshold)
		{
			Reflex = -1.0;   // for reflex action
			Penalty = -1.0; // for learning
		}
	}

	//std::cout << Reflex << " " << Penalty << "\n";
	if (!learning)
	{
		//Test ICO weight///
		//X=-0.1396263;TH=-0.1396263 (-8 degrees)
		//	kico[_X] = 0.671932;
		//	kico[_V] = 0.209382;
		//	kico[TH] = 1.108561;
		//	kico[OM] = 0.439055;


		//0,0
		//	kico[_X] = 0.074760;
		//	kico[_V] = 0.048776;
		//	kico[TH] = 0.317626;
		//	kico[OM] = 0.084527;

		//Combine learning 53 trials, -0.5, -9 deg//
		//	kico[_X] = 0.246126;
		//	kico[_V] = 0.097918;
		//	kico[TH] = 0.384245;
		//	kico[OM] = 0.123572;


		//Combine -2.0, -10 deg, 89 trials
		//	kico[_X] = 6.886477;
		//	kico[_V] = 1.501560;
		//	kico[TH] = 3.896624;
		//	kico[OM] = 1.736553;

		//initialX = 2.000000; initialV = 0.000000; initialTH = 10.999988; initialOM = 0.000000;
		//kico[_X] = 23.346636; kico[_V] = 5.980070; kico[TH] = 13.588497; kico[OM] = 4.208842;


		//initialX = -2.4; initialV = 0.000000; initialTH = 7; initialOM = 0.000000;

		//	 kico[_X] = 24.744239;
		//	 kico[_V] = 3.605136;
		//	 kico[TH] = 15.714474;
		//	 kico[OM] = 3.445053;

		//x_int =  1.400000, v_int = 0.000000, th_int = -0.191986 (-11.000000 deg), om_int = 0.000000

		// kico[_X] = 2.880537;
		//  kico[_V] = 0.966759;
		//  kico[TH] = 3.777650;
		//   kico[OM] = 1.722616;

		//x_int =  2.400000, v_int = 0.000000, th_int = -0.209440 (-12.000000 deg), om_int = 0.000000
		//Combinatorial learning

		kico[_X] = 0.988;
		kico[_V] = 0.145;
		for (int i = 2; i < XDIM; i+=2)
		{
			kico[i] = 0.978;
			kico[i+1] = 0.145;
		}
	}
	u_ico[0] = Reflex*1.0;
	for(int i = 0; i < XDIM; i++)
	{
		u_ico[0] += kico[i] * x_com[i];
	}
	//Find derivative of Reflex signal for learning/////////////////
	deri_Penalty = Penalty-Penalty_pre;

	//Only positive derivative for update weights
	deri_Penalty_actual = abs((deri_Penalty>0)? 0:deri_Penalty); // when define reflex (reward) = -1 => q = |min(0, delta_r(t))|
	if(stopExploration) // Stop exploration
		learningRate_ico = learningRate_ico_value;
	if(learning)
	{
		kico[_X] += learningRate_ico*deri_Penalty_actual*fabs(x_com[_X]);//(*/x_ico[_X]);
		kico[_V] += learningRate_ico*deri_Penalty_actual*fabs(x_com[_V]);//(*/x_ico[_V]);
		for(int i = 2; i < XDIM; i++)
		{
			kico[i] += learningRate_ico*deri_Penalty_actual*fabs(x_com[i]);
		}
	}
}

















newACICO::newACICO(const std::string& name, const std::string& revision): AbstractController(name, revision)
{
	/*
	 * Important Flags:
	 */
	useESN = true; //true; // else use RBF
	combination = 2; // 0: Use ICO, 1: use ACRL, else: use combination
	learningNoise = true ; //false; //false; // whether to use the Learning noise
	stopExploration = false; //true;
	learning = true;
	secondpole = false; //true; //false; //true; //false; //whether we use 1 or 2 poles

	if (secondpole)
		XDIM=6;//input dimensions
	else
		XDIM = 4;
	UDIM=1; //output dimensions
	/*
	 * Readability:
	 */
	_X = 0; //position
	_V = 1; //motion
	TH = 2; //angle
	OM = 3; //angular velocity
	/*
	 * Main step variables
	 */
	xt = new double[XDIM];//sensors
	xt_prev = new double[XDIM];// used for derivatives
	ut = new double[UDIM];//motors for AC
	utico =  new double[UDIM]; //motors for ICO

	xmin =  new double[XDIM];
	xmax =  new double[XDIM];


	/*
	 * AC variables
	 */
	basis = new int[XDIM];	//RBF internal stuff
	wbasis = new double[XDIM];
	k = new double[XDIM]; //initial weight values AC RL
	basis_width =  new double[XDIM];
	balance_time_log =  new double[MAX_TRIAL];
	sig_out_log =  new double[MAX_TRIAL];
	output_elig =  new double[XDIM];
	sig_out =  new double[UDIM];
	exp_output = new double[UDIM];
	sig_elig = new double[UDIM];
	output_grad = new double[UDIM];
	sig = new double[UDIM /*output number*/];


	ESN = new ESNetwork(XDIM+1/*+1*/,UDIM,50);

	ESN->generate_random_weights();

	ngnet = new NGNet();
	/*
	 * ICO variables
	 */
	kico = new double[XDIM]; //initial weight values ICO
	x_com = new double[XDIM];


	//Initializations
	simulations_runned = 0;
	nbasis = 0;
	for (int i = 0; i < XDIM; i++)
	{
		k[i] = 0; //initial weight values AC RL
		kico[i] = 0; //initial weight values ICO
	}
	output = 0.0;
	Vt = 0.0;
	reset = 0;
	simulations_run = 0;
	//-----ICO learning---//

	Reflex = 0.0,
			deri_Penalty_actual = 0.0, Penalty_pre = 0.0,
			Penalty = 0.0, deri_Penalty = 0.0;
	for (int i = 0; i < XDIM; i++)
	{
		x_com[i] = 0.0;
	}
	//-------------------//

	//stepcounter
	stepcounter = 0;
	//3)-----------Sensory input limit
	xt_limit = 2.37;//2.6;
	theta_limit = 0.2; /* -+0.2094*/
	rt_sum = 0;
	random_state_set = 0; // 1 = random // 0 = defined
	initialX = 0.0;
	initialV = 0.0;
	initialTH = 0.0 ;//0;
	initialOM = 0.0;
	acum_reward = 0.0;
	balance_time = 0.0;
	failure_flag = 1; //<--- initialize
	ntrial = 0;
	td_error = 0.0;
	save = 1;
	//control stuff
	dt = 0.01;
	sig_out[0]=0.0;
	si = 5; // output factor
	unit_activation_thresh = 0.6;
	error_thresh = 0.1;
	tau = 0.09;	/* value horizon */
	tau_elig = 0.01;	/* time const for critic */
	tau_elig_output = 0.05;	/* time const for critic */
	kappa    = 0.2;	/* time const for actor  */
	td_limit = 1.0;
	rate_valuefunction = 0.5;
	rate_policy = 0.5;//0.5;//0.001;//kimura
	gam = 0.95;
	lambda_v = 0.0; /*e trace of v value = 0.5 is good! = 190 nc ,no e trace needs about 240 nc*/
	lambda_p = 0.0; /*e trace of v value = 0.5 is good! = 190 nc ,no e trace needs about 240 nc*/
	learningRate_ico_value = 0.2;  //0.01;//0.01; 0.001
	learningRate_ico = 0.1;
	steps = 0;

	// Initial state////
	xt[_X /*input 0*/] = 0.0; //0
	xt[_V /*input 1*/] = 0.0;
 	xt[TH /*input 2*/] = 0.0; //0
	xt[OM /*input 3*/] = 0.0;


	xt[TH+2 /*input 4*/] = 0.0;
	xt[OM+2 /*input 5*/] = 0.0;

	initX = -0.5;
	initT = 0.0;
	finX = -0.0;
	finT = 0.0;
	sweepStepT = 0.04;
	sweepStepX = 0.25;
	indexX = initX;
	indexT = initT;



	alpha = 0.5;


	srand(time( NULL));

	/*CONTOL POLICY and AC learning*/
	init_funcapprox();//------------------------------ IMPPORTANT CAL CENTER & VARIENCE

	/*CONTOL POLICY and ICO learning*/
	//init_icolearning();


	addInspectableValue("sensorx",&xt[0],"position");
	addInspectableValue("sensorv",&xt[1],"velocity");
	addInspectableValue("sensort",&xt[2],"angle position");
	addInspectableValue("sensorw",&xt[3],"ang. vel.");
	//addInspectableValue("sensort2",&xt[4],"2 angle position");
	//addInspectableValue("sensorw2",&xt[5],"2 ang. vel.");
	addInspectableValue("icox",&x_com[_X],"position");
	addInspectableValue("icov",&x_com[_V],"velocity");
	addInspectableValue("icot",&x_com[TH],"angle position");
	addInspectableValue("icow",&x_com[OM],"ang. vel.");
	addInspectableValue("kx",&k[0],"position weight");
	addInspectableValue("kv",&k[1],"velocity weight ");
	addInspectableValue("kt",&k[2],"ang. position weight");
	addInspectableValue("ko",&k[3],"ang. vel. weight");
	//addInspectableValue("kt2",&k[4],"2 ang. position weight");
	//addInspectableValue("ko2",&k[5],"2 ang. vel. weight");
	addInspectableValue("kicox",&kico[0],"position weight");
	addInspectableValue("kicov",&kico[1],"velocity weight ");
	addInspectableValue("kicot",&kico[2],"ang. position weight");
	addInspectableValue("kicoo",&kico[3],"ang. vel. weight");
	//addInspectableValue("kicot2",&kico[4],"2 ang. position weight");
	//addInspectableValue("kicoo2",&kico[5],"2 ang. vel. weight");
	addInspectableValue("VT",&Vt,"Vt");
	addInspectableValue("tderr",&td_error,"tderror");
	addInspectableValue("rtsum",&rt_sum,"rt");
	addInspectableValue("ut",&ut[0],"AC output");
	addInspectableValue("utico",&utico[0],"ICO output");
	addInspectableValue("rtsum",&rt_sum,"rt");

}
/*
 *
 *
 * needs to go outside: 	simulation_loop();//---------------------- CALL CONTROLLER
 */
