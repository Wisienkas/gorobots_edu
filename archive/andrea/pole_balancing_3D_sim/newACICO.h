/*
 * newACICO.h
 *
 *  Created on: Dec 28, 2012
 *      Author: andrej
 */

#include <stdio.h>
#include "ngnet.h"
#include <fstream>
#include <selforg/randomgenerator.h>
#include <selforg/configurable.h>
#include <selforg/inspectable.h>
#include <selforg/storeable.h>
#include <selforg/abstractcontroller.h>
#include "networkmatrix.h"


#ifndef NEWACICO_H_
#define NEWACICO_H_


//4)-----------Clip function!!
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
/* Termination criterion. */
#define MAX_ITER  50000 // 100000 1000 [sec] //2000 // 20 [sec]
#define MAX_TRIAL 200//100000 //500
///----Select option:

//-- select one of this for driving cart-pole with ac, ico, or combination
//#define	acrl
//#define ico
//#define combination



//for noise adaptive reduction of ac [good when max reward = 0, and punishment = -1 is given]
//#define learningnoise //comment this to use Doya noise reduction

//for actor critic learning

#define stopafter 101//11 //101
#define number_of_reset 2//21//21//10001


class newACICO: public AbstractController
{
	typedef double sensor;
	typedef double motor;


public:
	newACICO(const std::string& name, const std::string& revision);

	void init_funcapprox();
	void exploration_grad(double *exp_output, double *sig_elig, double *sig, double si);
	void exploration_output(double *sig, double si, double *x);
	void output_policy(double *x, double *u);
	void update_policy(double td_error, double rate);
	void update_policy_trace(double *xt, double lambda);
	double reward_function(double *x, double *u);
	double value_function(double *x);
	void update_valuefunction( double *x, double td, double rate);
	void update_valuefunction_trace(double *x);
	double gauss();

	void init(int sensornumber, int motornumber, RandGen* randGen = 0){}

	void init_icolearning();



	int check_limit(double *x);
	double uniform_dist()
	{
		double value;
		double max_t = 1;
		double min_t = -1;

		value = ((max_t-min_t)*((double)rand()/RAND_MAX))+min_t; //Range [-1,...,+1]

		//2*((double)rand()/(RAND_MAX))-1.0;
		//printf("value %f \n", value);

		return value;
	}
	void reset_state(double *x);


	void step(const sensor* sensors, int sensornumber, motor* motors, int motornumber);
	void stepNoLearning(const sensor* sensors, int sensornumber, motor* motors, int motornumber)
	{
	}

	void output_icolearning(double *x_ico, double *u_ico);
	void reset_simulation();
	int simulations_runned;
	/*
	 * IMPORTANT interface for LPZrobots, so LPZ can use this controller directly
	 * */

	/// returns the number of sensors the controller was initialised with or 0 if not initialised
	virtual int getSensorNumber() const { return XDIM; }
	/// returns the mumber of motors the controller was initialised with or 0 if not initialised
	virtual int getMotorNumber() const  { return UDIM; }
	/*coordinates*/
	/**** STOREABLE ****/
	/** stores the controller values to a given file. */
	virtual bool store(FILE* f) const { return true; };
	/** loads the controller values from a given file. */
	virtual bool restore(FILE* f){ return true;};


	//Very important stuff
	bool useESN;
	int combination;
	bool learningNoise;
	bool stopExploration;
	bool learning;
	bool secondpole;
	int _X; //0 //position
	int _V; //1 //motion
	int TH; //2 //angle
	int OM; //3 //angular velocity
	int XDIM;// 4 //input dimensions
	int UDIM; //1 //output dimensions
	//NGNET
	NGNet* ngnet;

	//ESN
//	ArmaESN *ESN;

	ESNetwork * ESN;

	FILE* fdatadebug;

	double *xt;//sensors
	double *xt_prev;// used for derivatives
	double *ut;//motors
	double *utico;

	/*
	 * IMPORTANT STUFF, SORT THROUGH IT LATER
	 * */
	//******RBF***********//
	int nbasis;
	int *basis;
	double *wbasis;
	//*******************//

	//double k[XDIM] = {0.655846,  0.935163,  13.996960, 6.401443}; //initial weight values
	double *k; //initial weight values AC RL
	double *kico; //initial weight values ICO
	double output;
	double Vt;
	int reset;
	int simulations_run;
	//-----ICO learning---//

	double Reflex, deri_Penalty_actual, Penalty_pre, Penalty, deri_Penalty;
	//double x_com, theta_com, x_dot_com, theta_dot_com;
	//double theta_two_com, theta_two_dot_com;
	double *x_com;
	//added here

	//-------------------//

	//stepcounter
	int stepcounter;
	std::ofstream fileOutput;


	//3)-----------Sensory input limit
	double xt_limit;//2.6;
	double theta_limit; /* -+0.2094*/

	double *xmin;
	double *xmax;
	double *basis_width;
	double sig_gain;

	Cell VALUE;
	double rt_sum;





	// if not scan and random
	int random_state_set; // 1 = random // 0 = defined
	double initialX;
	double initialV;
	double initialTH;
	double initialOM;
	/*************************************************************
	 *	CALL learning and controller repeat until stop!!
	 *************************************************************/

	double acum_reward;
	double balance_time;

	//double ** param_log;
	double *balance_time_log;
	double *sig_out_log;
	int failure_flag; //<--- initialize
	int ntrial;


	double  td_error;

	int save;
	FILE *fdata;
	FILE *fdata2;
	FILE *fdataico;

	//control stuff
	double dt;
	double * output_elig;
	double *sig_out;
	double *exp_output;
	double si; // output factor


	double unit_activation_thresh;
	double error_thresh;

	double	tau;	/* value horizon */
	double	tau_elig;	/* time const for critic */
	double	tau_elig_output;	/* time const for critic */
	double	kappa;	/* time const for actor  */


	double  td_limit;
	double  *sig_elig;
	double  *output_grad;
	double  rate_valuefunction;
	double  rate_policy;//0.5;//0.001;//kimura
	double  gam;
	double  lambda_v; /*e trace of v value = 0.5 is good! = 190 nc ,no e trace needs about 240 nc*/
	double  lambda_p; /*e trace of v value = 0.5 is good! = 190 nc ,no e trace needs about 240 nc*/


	double learningRate_ico_value;  //0.01;//0.01; 0.001
	double learningRate_ico;

	/*************************************************************
	 *	ACTOR
	 *************************************************************/
	double *sig;
	double exploration_g;
	double exploration_lowpass_old_g;
	double exploration_lowpass_g;
	double Vmax;
	double Vmin;
	double y;
	double max_value;
	double min_value;
	double alpha; // weight of ICO output
	double initX, initT, finX, finT, sweepStepT, sweepStepX, indexX, indexT; //to sweep the parameters between these value
	//Set initial state option
	int steps;
	int total_trials;
	int iter;
	int i;
	double  rt;//, Vt;
	//double  td_error = 0.0;
	double Vt_old;
	FILE *fp;
	int m;
	int *p1;



};


#endif /* NEWACICO_H_ */



























