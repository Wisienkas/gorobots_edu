/***********************************************/
/*                                             */
/*  cart_pole.c                                */
/*                                             */
/*  Jun Morimoto 2010.6.5                      */
/*                                             */
/*  Limited use for the collaboration study    */
/*  between Univ. of Gottingen and ATR-BRI     */
/*                                             */
/***********************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include "NGnet.h"
#include "cart_pole.h"

#define N_RKBUFF 7



/*************************************************************
 *	Define variables and class
 *************************************************************/


//1)----------- Simulation cart_pole
void cart_pole(int n, double t, double *x, double *xdot); 

//2)-----------Runge-Kutta method for cart pole simulation
void rkstep( void (*vf)(),	
    int n, double *y,
    double *pt, double dt,
    double *work);

double cp_time = 0.0;
double dt = 0.01;
double rk_buffer[XDIM*N_RKBUFF /*4x7 = 28 arrays*/];
int nloop = 1000000;

int xdim = XDIM /*4 inputs*/;
double force=0.0;



//******RBF***********//
int nbasis = 0;
int basis[XDIM/*=4 inputs*/];
double wbasis[XDIM/*=4 inputs*/];
//*******************//

//double k[XDIM] = {0.655846,  0.935163,  13.996960, 6.401443}; //initial weight values
double k[XDIM] = {0.0,  0.0,  0.0,  0.0}; //initial weight values AC RL
double kico[XDIM] = {0.0,  0.0,  0.0,  0.0}; //initial weight values ICO
double output = 0.0;
double Vt = 0.0;
int reset = 0;

//-----ICO learning---//

double Reflex = 0.0,
    deri_Penalty_actual = 0.0, Penalty_pre = 0.0,
    Penalty = 0.0, deri_Penalty = 0.0;
double x_com = 0.0, theta_com = 0.0, x_dot_com = 0.0, theta_dot_com = 0.0;
//-------------------//





//3)-----------Sensory input limit
double xt_limit = 2.4;//2.6;
double theta_limit = 12.0/180.*M_PI; /* -+0.2094*/

double xmin[XDIM /*=4 inputs*/];
double xmax[XDIM /*=4 inputs*/];
double basis_width[XDIM /*=4 inputs*/];
double sig_gain;

Cell VALUE;

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


///----Select option:

//-- select one of this for driving cart-pole with ac, ico, or combination
//#define	acrl
//#define ico
#define combination


#define animation //comment this to run without animation
#define learning //comment this to test the learned weights


//for noise adaptive reduction of ac [good when max reward = 0, and punishment = -1 is given]
#define learningnoise //comment this to use Doya noise reduction

//for actor critic learning
#define stopexploration
#define stopafter 1//11 //101
#define number_of_reset 2//21//21//10001


// if not scan and random
static int random_state_set = 0; // 1 = random // 0 = defined
double initialX = 2.0;
double initialV = 0.0;
double initialTH = 0.1919860;
double initialOM = 0.0;


//Non random position
// -0.0174533; // -1 degrees
// -0.0349066; // -2 degrees
// -0.0523599; // -3 degrees
// -0.0698132; // -4 degrees
// -0.0872665; // -5 degrees
// -0.1047198; // -6 degrees
// -0.1221730; // -7 degrees
// -0.1396263; // -8 degrees
// -0.1570796; // -9 degrees
// -0.1745329; // -10 degrees
// -0.1919860; // -11 degrees
// -0.2094395; // -12 degree

/*************************************************************
 *	RBF network
 *************************************************************/
void init_funcapprox()
{
  int i;
  int state_index[XDIM /*input, e.g., 4 or 2*/];
  double xc[XDIM /*input, e.g., 4 or 2*/];

  int t = 0;

  int *p1, *p2;
  //  FILE *fp;

  //Function!!
  init_incsbox(&VALUE, xdim /*4 inputs*/,  1 /*output*/);
  reset_incsbox(&VALUE);
  //

  basis[_X/*0*/] = 3; // Number of RBF
  basis[_V/*1*/] = 3; // Number of RBF
  basis[TH/*2*/] = 6; // Number of RBF
  basis[OM/*3*/] = 3; // Number of RBF

  //1) Input limitation MAX MIN------------------------------------------------CLEAR
  xmax[_X] = xt_limit; // 2.6 m, KOH 2.4  NOT
  xmin[_X] = -xt_limit; // -2.6 m, KOH -2.4  NOT

  xmax[_V] = 3.5;//26.0; //2.6x10 velocity, KOH 3.5 NOT
  xmin[_V] =-3.5;//-26.0; //-2.6x10 velocity, KOH -3.5 NOT

  xmax[TH] = theta_limit; // 12/180*PI = 12 deg =   0.2094 rad EQ
  xmin[TH] =-theta_limit; // -12/180*PI = -12 deg = -0.2094 rad EQ

  xmax[OM] = 2.0;//180.0/180.0*M_PI; //3.1416 rad/s, KOH 2.0 NOT
  xmin[OM] =-2.0;//-180.0/180.0*M_PI; //-3.1416 rad/s, KOH -2.0 NOT



  //  fp = fopen("basis_center.dat","a");

  //2) find width of each input ------------------------------------------------CLEAR
  nbasis = 0; // reset
  for(i=0;i<xdim /*4 inputs*/;i++)
  {
    basis_width[i]  = (xmax[i] - xmin[i])/(2.0*basis[i]/*number of centers*/);
    wbasis[i] = 1.0/basis_width[i]; //width of each input

    //		wbasis[0] = ivar[0] = 1.250000
    //		wbasis[1] = ivar[1] = 0.857143
    //		wbasis[2] = ivar[2] = 28.647890
    //		wbasis[3] = ivar[3] = 1.500000

  }

  //3) calculate center of neuron (162 neurons)-------------------------------------CLEAR
  for(state_index[_X /*0*/]=0; state_index[_X]<basis[_X] /*3*/; state_index[_X]++) // X
  {
    printf("loop0\n");
    for(state_index[_V /*1*/]=0; state_index[_V]<basis[_V] /*3*/; state_index[_V]++) // V
    {
      printf("loop1\n");
      for(state_index[TH /*2*/]=0; state_index[TH]<basis[TH] /*6*/; state_index[TH]++) // TH
      {
        printf("loop2\n");
        for(state_index[OM /*3*/]=0;state_index[OM]<basis[OM] /*3*/; state_index[OM]++) // OM
        {
          printf("loop3\n");
          for(i=0;i<xdim /*4 inputs*/;i++)
          {
            printf("loop4\n");
            xc[i]  = (xmin[i]+basis_width[i])+state_index[i]*2.0*basis_width[i];

            //xc[0], xc[1], xc[2], xc[3]
            //t=162 neurons = 3x3x6x3

            p1 = &nbasis;
            printf("xc [%d] = %lf  t= %d nbasis = %d\n",i, xc[i],t++,*p1);

            //fprintf(fp,"%lf ",xc[i]);
          }
          printf("\n");
          //fprintf(fp,"\n");
          put_incsbox(&VALUE, xdim, 1, xc, wbasis, &nbasis);

        }

      }

    }


  }


  //  fclose(fp);

}

/*************************************************************
 *	End RBF network
 *************************************************************/


/*************************************************************
 *	ICO network
 *************************************************************/
void init_icolearning()
{
  printf("activate ico learning \n\n");
  //Input limitation MAX MIN
  xmax[_X] = xt_limit; // 2.6 m, KOH 2.4  NOT
  xmin[_X] = -xt_limit; // -2.6 m, KOH -2.4  NOT

  xmax[_V] = 3.5;//26.0; //2.6x10 velocity, KOH 3.5 NOT
  xmin[_V] =-3.5;//-26.0; //-2.6x10 velocity, KOH -3.5 NOT

  xmax[TH] = theta_limit; // 12/180*PI = 12 deg =   0.2094 rad EQ
  xmin[TH] =-theta_limit; // -12/180*PI = -12 deg = -0.2094 rad EQ

  xmax[OM] = 2.0;//180.0/180.0*M_PI; //3.1416 rad/s, KOH 2.0 NOT
  xmin[OM] =-2.0;//-180.0/180.0*M_PI; //-3.1416 rad/s, KOH -2.0 NOT

}


int main(int argc, char **argv)
{

  // Initial state////
  xt[_X /*input 0*/] = 0.0;
  xt[_V /*input 1*/] = 0.0;
  xt[TH /*input 2*/] = 0.0;
  xt[OM /*input 3*/] = 0.0;


  printf("Initial state:  xt = %f,  x_dot = %f,  th = %f, th_dot = %f\n", xt[_X /*input 0*/], xt[_V /*input 1*/], xt[TH /*input 2*/], xt[OM /*input 3*/]);

  //printf("clip %f.\n",clip(-3.9,-1,1));

  srand(time( NULL));

  /*CONTOL POLICY and AC learning*/
  init_funcapprox();//------------------------------ IMPPORTANT CAL CENTER & VARIENCE

  /*CONTOL POLICY and ICO learning*/
  //init_icolearning();

#ifdef animation

  printf("ANIMATION");
  /*Simulation animation NO CONTROLLER ACTIVATED!*/
  gl_init_anim(argc, argv);

  /* GLUT main loop  */
  glut_loop();//-------->> call   simulation_loop();--- CALL CONTROLLER

#endif

#ifndef animation
  printf("NO ANIMATION");
  simulation_loop();//------------------------------------------------------------- CALL CONTROLLER
#endif

  return 0;
}

double sig_out[UDIM]={0.0};
double si = 5; // output factor
double exp_output[UDIM];

double unit_activation_thresh = 0.6;
double error_thresh = 0.1;

double	tau = 0.09;	/* value horizon */
double	tau_elig = 0.01;	/* time const for critic */
double	tau_elig_output = 0.05;	/* time const for critic */
double	kappa    = 0.2;	/* time const for actor  */


double  td_limit = 1.0;
double  sig_elig[UDIM];
double  output_grad[UDIM];
double  rate_valuefunction = 0.5;
double  rate_policy = 0.5;//0.5;//0.001;//kimura
double  gam = 0.95;
double  lambda_v = 0.0; /*e trace of v value = 0.5 is good! = 190 nc ,no e trace needs about 240 nc*/
double  lambda_p = 0.0; /*e trace of v value = 0.5 is good! = 190 nc ,no e trace needs about 240 nc*/


double learningRate_ico_value = 0.1;  //0.01;//0.01; 0.001
double learningRate_ico = 0.1;

/*************************************************************
 *	Gaussian random variable: just a sum of uniform distribution
 *	Average = 0, Variance = 1
 *************************************************************/

double gauss()
{
  double	sum;
  int 	i;

  for( sum = i = 0; i < 12; i++) sum += (1.0*(double)rand()/(RAND_MAX));
  return( sum - 6.0);
}


/*************************************************************
 *	EXPLORATION Gradient
 *************************************************************/
double output_elig[XDIM]={0.0, 0.0, 0.0, 0.0};
void exploration_grad(double *exp_output, double *sig_elig, double *sig, double si)
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
}//after 10 resets


/*************************************************************
 *	EXPLORATION
 *************************************************************/
void exploration_output(double *sig, double si, double *x)
{
  int i;

  for(i=0;i<UDIM;i++)
  {

    //Morimoto
    sig[i] = si/(1+exp(-sig_out[i]));

    //Kimura
    //sig[i] = si/(1+exp(-sig_out[i]));
    //printf("sig [0] = %f si = %f sig_out = %f \n", sig[i],si,sig_out[i]);
  }

}


/*************************************************************
 *	ACTOR
 *************************************************************/
double sig[UDIM /*output number*/];
double exploration_g;
double exploration_lowpass_old_g;
double exploration_lowpass_g;
double Vmax;
double Vmin;
double y;
double max_value;
double min_value;


void output_policy(double *x, double *u)
{

  int i;
  double u_tmp = 0.0;
  double sig0_nolearning;
  double th_exp_td_vt = 0.05;


#ifdef learning

  for(i=0;i<XDIM /*input numbers*/;i++)
  {
    //u_tmp += (-(k[i])*x[i]);
    u_tmp += (-(k[i])*x[i])*-1;
  }

  exploration_output(sig, si /*10*/, x);

  //Low pass filer noise NOT USED
  double lp_gain = 0.98;
  exploration_g = gauss();
  exploration_lowpass_old_g = exploration_lowpass_g;
  exploration_lowpass_g = exploration_lowpass_old_g*lp_gain+(1-lp_gain)*exploration_g;




#ifdef learningnoise // Morimoto Work best

#ifdef stopexploration // Stop exploration
  if(reset>=stopafter)
    exp_output[0]= 0.0;
  else//if(reset<stopafter)
    exp_output[0]= sig[0]*gauss();//*0.5;//exploration_lowpass_g;
#endif

#ifndef stopexploration // Not stop exploration
  exp_output[0]= sig[0]*gauss();//exploration_lowpass_g;
#endif

#endif




#ifndef learningnoise // Doya NOT work
  Vmax = 0.0;// Max expected value//---------------------------------**** Change
  Vmin = -1.0;// Min expected value//---------------------------------****  Change
  sig0_nolearning = 5.0;//1.5;//si;

  y = (Vmax-Vt)/(Vmax-Vmin);
  max_value = max(0, y);
  min_value = min(1,max_value);
  sig_gain = sig0_nolearning*min_value;

  if(abs(sig_gain)<th_exp_td_vt)
    sig_gain = 0.0;

  exp_output[0]= sig_gain*gauss();//exp_output_decay*sig_nolearning*exploration_lowpass_g[i]*(lp_gain);
  printf("NO learning NOISE MODE \n");
#endif


  // Output to control cart!
  u[0] = u_tmp+exp_output[0];

#endif


#ifndef learning

  u[0] = k[0]*x[0]+k[1]*x[1]+k[2]*x[2]+k[3]*x[3];

  printf("TESTING MODE AC RL\n\n");

#endif

  output = u[0];

}

/*************************************************************
 *	UPDATE ACTOR WEIGHTS
 *************************************************************/
void update_policy(double td_error, double rate)
{
  int i;

  //Calculate adaptive noise //Morimoto
  sig_out[0] += rate*td_error*sig_elig[0];

#ifdef learning
  //Calculate weights
  for(i=0;i<XDIM /*4 inputs = 4 weights*/;i++)
    k[i] += rate*td_error*output_elig[i]; // Actor weights


  //--NOT important ----add koh limit weights
  int thershold_k = 50;

  for(i=0;i<XDIM /*4 inputs = 4 weights*/;i++)
  {
    if(abs(k[i]) > thershold_k)
      k[i]  = 0.0;
  }


#endif

#ifndef learning
  //2 time reset
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

  k[0] = -45.015554;
  k[1] = -11.723170;
  k[2] = 12.608269;
  k[3] = 26.428016;




#endif

  //	printf("sig_elig[0] = %f : sig_out = %f td_error = %f\n", sig_elig[0],sig_out[0],td_error);
}

/*************************************************************
 *	ETRACE of input x output
 *************************************************************/
void update_policy_trace(double *xt, double lambda)
{
  int i;

  for(i=0;i<XDIM;i++)
  {
    output_elig[i] *= lambda;
    output_elig[i] += output_grad[0]*xt[i]; /*output x input*/
    //output_elig[i] -= output_grad[0]*xt[i]; /*output x input*/
  }
}

/*************************************************************
 *	CALCULATE REWARD
 *************************************************************/
double reward_function(double *x, double *u)
{
  double r=0.0;
  double bias_xt = 0.0;//0.5;
  double bias_theta = 0.0;//0.0175;

  if(x[_X] < -(xt_limit-bias_xt) /* -2.4 deg*/ || x[_X] > (xt_limit-bias_xt) /* 2.4 deg*/)
    r = -1.0;//+= -1.0;

  if(x[TH] < -(theta_limit-bias_theta) /* -12 deg*/ || x[TH] > (theta_limit-bias_theta) /* +12 deg*/)
    r = -1.0;//+= -1.0;

  return r;


}

double value_function(double *x)
{
  double Value;

  incsbox_output(&VALUE, x, &Value, &nbasis);

  //printf("Value %f \n", Value);

  return Value;
}

void update_valuefunction( double *x, double td, double rate)
{
  double td_error_tmp[1];//, vt, yt[1];

  td_error_tmp[0] = -td;

  //	printf("double error_thresh=0.1; %f\n", error_thresh);
  //	printf("wbasis; 0= %f ; 1= %f; 2= %f; 3=  %f\n", wbasis[0], wbasis[1], wbasis[2], wbasis[3]);
  //	printf("unit_activation_thresh; %f\n", unit_activation_thresh);
  //	printf("&nbasis %d \n", nbasis);

  incsbox_update( &VALUE, x, td_error_tmp, rate, &nbasis, wbasis, error_thresh, unit_activation_thresh);
}

void update_valuefunction_trace(double *x)
{

  //printf("%f\n", lambda_v);
  incsbox_trace(&VALUE, x,lambda_v/*No trace if set to 0.0*/, &nbasis);
}

int check_limit(double *x)
{
  int flag=0;

  if(x[_X] < -xt_limit || x[_X]  > xt_limit)
  {
    flag = 1;
  }

  if(x[TH] < -theta_limit || x[TH] > theta_limit)
  {
    flag = 1;
  }

  return flag;

}


/*************************************************************
 *	RESET INITIAL STATE after fail!!!!
 *************************************************************/
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

void reset_state(double *x)
{
  int i;

  for(i=0;i<XDIM;i++)
    xt[i] = 0.0;

  xt[_X] = 0.5*xt_limit*uniform_dist(); // 0.5*2.4*(+1 or -1) = half of limit
  xt[TH] = 0.5*theta_limit*uniform_dist(); // 0.5*0.2094*(+1 or -1) = half of limit

  //xt[_X] = 1.1*xt_limit*uniform_dist(); // 0.5*2.4*(+1 or -1) = half of limit
  //xt[TH] = 1.1*theta_limit*uniform_dist(); // 0.5*0.2094*(+1 or -1) = half of limit


}




/*************************************************************
 *	CALL learning and controller repeat until stop!!
 *************************************************************/

static double acum_reward = 0.0;
static double balance_time = 0.0;
/* Termination criterion. */
#define MAX_ITER  100000 // 100000 1000 [sec] //2000 // 20 [sec]
#define MAX_TRIAL 1000//100000 //500


//#define MAX_FAILURES     100000
//#define MAX_STEPS        100000

double balance_time_log[MAX_TRIAL];
double sig_out_log[MAX_TRIAL];
static int failure_flag = 1; //<--- initialize
int ntrial = 0;

double  param_log[MAX_TRIAL][XDIM];
double  td_error = 0.0;

static int save = 1;
FILE *fdata;
FILE *fdata2;
FILE *fdataico;

//Set initial state option
int steps = 0;
int total_trials;

void simulation_loop()
{

  static int iter = 0;
  int i;
  double  rt;//, Vt;
  //double  td_error = 0.0;
  static double Vt_old = 0.0;
  FILE *fp;
  int m;
  int *p1;

  if(save)
  {
    fdata = fopen("save_data.dat","w");
    fdata2 = fopen("save_data2.dat","w");
    fdataico = fopen("save_dataico.dat","w");
    save = 0;
  };


#ifndef animation
  while(steps++ < MAX_ITER && ntrial < MAX_TRIAL && reset < number_of_reset)
  {
#endif

    /************************AFTER RESET********************************************************/
    //if(failure_flag || iter >= MAX_ITER) // reset every curtain time even it can balance in order to file optimal policy
    if(failure_flag || iter >= MAX_ITER/2)
    {
      if(iter >= MAX_ITER/2)
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

      }


      // RESET values
      failure_flag = 0;
      Vt_old = 0.0;
      //ntrial++;
      iter = 0;
      steps = 0;

      //Save data to analyze//
      acum_reward = 0.0; // RESET acc reward
      //balance_time_log[ACRLntrial] = balance_time;
      sig_out_log[ntrial] = sig_out[0];
      balance_time = 0.0;
      //printf("number of trial %d \n", ntrial);
    }


    /************************DURING LEARNING STATE************************************************/
    //if(ntrial < MAX_TRIAL)
    if(steps++ < MAX_ITER && ntrial < MAX_TRIAL && reset < number_of_reset)
    {

      /* Inputs of the system*/
      printf("x =  %f, v = %f, th = %f, om = %f\n\n", xt[_X], xt[_V], xt[TH], xt[OM]);



#ifdef ico
      //-------ICO learning--------------//
      output_icolearning(xt, utico);
      //--------------------------------//
      //ICO learning
      force = 10.*utico[0];
#endif


#ifdef acrl
      //-Actor critic reinforcement learning-//
      output_policy(xt, ut);
      //--------------------------------//
      //AC learning
      force = 10.*ut[0];
#endif

#ifdef combination

      //-ico and acRL learning--//
      //if(ntrial>100)
      output_policy(xt, ut);
      //else
      //  ut[0] = 0.0;

      output_icolearning(xt, utico);


      force = 10.*(0.5*ut[0]+0.5*utico[0]);

#endif

      /* Cart Pole Dynamics */
      rkstep(&cart_pole, xdim, xt, &cp_time, dt, rk_buffer);

      /* Check state limit */
      failure_flag = check_limit(xt);


      //if(ntrial>100)
      //{


        /* Cart Pole Dynamics */
        //rkstep(&cart_pole, xdim, xt, &cp_time, dt, rk_buffer);
        /* Reward */
        rt = reward_function(xt,ut); //return -1

        balance_time += dt;

        /* Check state limit */
        //failure_flag = check_limit(xt);

        /* Value function estimation for current state */
        Vt = value_function(xt);
        //printf("Result Vt %f \n", Vt);


        if(failure_flag)
        {
          //Vt = 0;!!
          td_error = rt - Vt_old;
          td_error = clip(td_error, -td_limit /*-1*/, td_limit /*1*/); /// CLIP limit TD error
        }
        else
        {
          td_error = rt + gam*Vt - Vt_old;
          td_error = clip(td_error, -td_limit /*-1*/, td_limit /*1*/); /// CLIP limit TD error
          Vt_old = Vt;
        }
        //-----------------------------------------------------------------------------------
        /* gradients for policy parameters*/
        exploration_grad(exp_output, sig_elig, sig, si);
        //printf("sig_out[0] %f  sig[0] %f", sig_out[0], sig[0]);


        for(i=0;i<UDIM;i++)
        {
#ifdef learningnoise
          //Morimoto
          output_grad[i] = clip(exp_output[i]/(sig[i]*sig[i]),-1.0, 1.0); // with noise learning
#endif

#ifndef learningnoise
          //Doya & no learning noise
          output_grad[i] = clip(exp_output[i],-1.0, 1.0); // with noise learning
#endif
          //Kimura
          //output_grad[i] = clip(exp_output[i]/(sig[i]),-1.0, 1.0); // with noise learning
        //}




        /* Update parameters for value function approximator */
        update_valuefunction_trace(xt);
        update_valuefunction(xt, td_error, rate_valuefunction);

        /* Update parameters for policy */
        update_policy_trace(xt,lambda_p);
        update_policy(td_error, rate_policy);


        //Save data
        //AC RL
        p1 = &nbasis;

      }
      total_trials = ntrial+reset;
      //ACRL
      fprintf(fdata,"%d %f %f %f %f %f %f %f %f %d %d %d\n",total_trials, td_error, rt, Vt, sig[0], exp_output[0],  sig_elig[0], (exp_output[0]*exp_output[0] - sig[0]*sig[0]), (1 - (sig[0]/si))/(sig[0]*sig[0]), *p1, ntrial, reset);
      fprintf(fdata2,"%d %f %f %f %f %f %f %f %f %f %d\n",total_trials, xt[_X /*input 0*/],xt[_V /*input 1*/], xt[TH /*input 2*/],xt[OM /*input 3*/], output, k[0], k[1], k[2], k[3],ntrial);
      //ICO
      fprintf(fdataico,"%d %f %f %f %f %f %f %f %f %f %d\n",total_trials, xt[_X /*input 0*/],xt[_V /*input 1*/], xt[TH /*input 2*/],xt[OM /*input 3*/],utico[0], kico[0], kico[1], kico[2], kico[3],ntrial);

      iter++;

#ifdef ico

      printf("----ICO learning----- \n\n");
      if(random_state_set)
      {
        printf("//RANDOM Initial state:: xt[_X] = %f, xt[_V] = %f, xt[TH] = %f, xt[OM] = %f\n", initialX, initialV,initialTH*180/M_PI,initialOM);
      }
      else
      {
        printf("//Defined Initial state:: xt[_X] = %f, xt[_V] = %f, xt[TH] = %f, xt[OM] = %f\n", initialX, initialV,initialTH*180/M_PI,initialOM);
      }

      printf("kico[_X] = %f; kico[_V] = %f; kico[TH] = %f; kico[OM] = %f;\n", kico[_X],kico[_V],kico[TH],kico[OM]);
      printf("Trials %d nbasis = %d\n", ntrial, *p1);
      printf("steps %d :: iter %d: total trials %d :: reset %d\n", steps, iter, total_trials, reset);
#endif

#ifdef acrl

      printf("-----AC RL----------- \n\n");
      if(random_state_set)
      {
        printf("//RANDOM Initial state:: xt[_X] = %f, xt[_V] = %f, xt[TH] = %f, xt[OM] = %f\n", initialX, initialV,initialTH*180/M_PI,initialOM);
      }
      else
      {
        printf("//Defined Initial state:: xt[_X] = %f, xt[_V] = %f, xt[TH] = %f, xt[OM] = %f\n", initialX, initialV,initialTH*180/M_PI,initialOM);
      }
      printf("k[_X] = %f; k[_V] = %f; k[TH] = %f; k[OM] = %f;\n", k[_X],k[_V],k[TH],k[OM]);
      printf("Trials %d nbasis = %d\n", ntrial, *p1);
      printf("steps %d :: iter %d: total trials %d :: reset %d\n", steps, iter, total_trials, reset);
#endif

#ifdef combination

      printf("---------Combinatorial learning--------- \n\n");
      if(random_state_set)
      {
        printf("//RANDOM Initial state:: xt[_X] = %f, xt[_V] = %f, xt[TH] = %f, xt[OM] = %f\n", initialX, initialV,initialTH*180/M_PI,initialOM);
      }
      else
      {
        printf("//Defined Initial state:: xt[_X] = %f, xt[_V] = %f, xt[TH] = %f, xt[OM] = %f\n", initialX, initialV,initialTH*180/M_PI,initialOM);
      }
      printf("k[_X] = %f; k[_V] = %f; k[TH] = %f; k[OM] = %f;\n", k[_X],k[_V],k[TH],k[OM]);
      printf("kico[_X] = %f; kico[_V] = %f; kico[TH] = %f; kico[OM] = %f;\n", kico[_X],kico[_V],kico[TH],kico[OM]);
      printf("Trials %d nbasis = %d\n", ntrial, *p1);
      printf("steps %d :: iter %d: total trials %d :: reset %d\n", steps, iter, total_trials, reset);
#endif


    }
    else
    {

      ////---------Print success or fail--------------////
      if (ntrial == MAX_TRIAL)
      {
        //TimeElapsed = 100.0;
        printf("\n");
        printf("Pole not balance %d failures\n",ntrial); //* 180/pi

      }

      else
      {
        //TimeElapsed = ((double)clock() - start) / CLOCKS_PER_SEC;
        printf("\n");
        printf("Pole balanced successfully for at least %d steps \n", steps);
        printf("k[0] = %f; \n k[1] = %f; \n k[2] = %f; k[3] = %f; \n ", k[0], k[1], k[2], k[3]);

        //printf("Time elapsed: %f\n", TimeElapsed); // Time function
      }

      ////---------Print success or fail--------------////
      exit(0);
    }


#ifndef animation
  }
#endif

}


/*************************************************************
 *	ICO learning
 *************************************************************/

void output_icolearning(double *x_ico, double *u_ico)
{


  double maxX = 2.4; //2.6 morimoto
  double minX = -2.4;//-2.6 morimoto

  double maxtheta = 0.2094384;
  double mintheta = -0.2094384;

  double maxX_dot = 3.5; //26 morimoto
  double minX_dot = -3.5; //-26 morimoto

  double maxtheta_dot = 2.0;
  double mintheta_dot = -2.0;

  //To scale to between +-1



  x_com = ((x_ico[_X]-minX)/(maxX-minX))*2-1;

  if(x_com>1)
    x_com = 1;
  if(x_com<-1)
    x_com = -1;

  theta_com = ((x_ico[TH]-mintheta)/(maxtheta-mintheta))*2-1;

  if(theta_com>1)
    theta_com = 1;
  if(theta_com<-1)
    theta_com = -1;

  x_dot_com = ((x_ico[_V]-minX_dot)/(maxX_dot-minX_dot))*2-1;

  if(x_dot_com>1)
    x_dot_com = 1;
  if(x_dot_com<-1)
    x_dot_com = -1;

  theta_dot_com =  ((x_ico[OM]-mintheta_dot)/(maxtheta_dot-mintheta_dot))*2-1;

  if(theta_dot_com>1)
    theta_dot_com = 1;
  if(theta_dot_com<-1)
    theta_dot_com = -1;

  // Remember the old state
  Penalty_pre =  Penalty;


  //#### (1)
  // Setting reflex
  // Reflex action is trigger
  // when theta > +-0.2007 rad (+-11.5 deg)
  // or when x > +- 2.35 m

  if(x_ico[TH] > /*0.196*/ 0.2007  || x_ico[_X] < -2.35)
  {
    Reflex = 1.0;   // for reflex action
    Penalty = -1.0; // for learning
  }
  else if(x_ico[TH] < /*-0.196*/ -0.2007  ||  x_ico[_X] > 2.35 )
  {
    Reflex =  -1.0; // for reflex action
    Penalty = -1.0; // for learning
  }
  else
  {
    Reflex = 0.0; // for reflex action
    Penalty = 0.0; // for learning
  }


#ifndef learning

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

  kico[_X] = 124.375805;
  kico[_V] = 79.330669;
  kico[TH] = 85.381904;
  kico[OM] = 10.041439;

  ///////////////////reset

  printf("TESTING MODE ICO \n\n");

#endif

  u_ico[0] = Reflex*1.0+kico[_X]*x_com+kico[_V]*x_dot_com+kico[TH]*theta_com+kico[OM]*theta_dot_com;
  //u_ico[0] = Reflex*1.0+kico[_X]*x_ico[_X]+kico[_V]*x_ico[_V]+kico[TH]*x_ico[TH]+kico[OM]*x_ico[OM];

  //Find derivative of Reflex signal for learning/////////////////
  deri_Penalty = Penalty-Penalty_pre;

  //Only positive derivative for update weights
  deri_Penalty_actual = abs((deri_Penalty>0)? 0:deri_Penalty); // when define reflex (reward) = -1 => q = |min(0, delta_r(t))|


#ifdef stopexploration // Stop exploration
  if(reset>=stopafter)
    learningRate_ico = 0.0;
  else
    learningRate_ico = learningRate_ico_value;
#endif




#ifdef learning
  kico[_X] += learningRate_ico*deri_Penalty_actual*fabs(x_com);//(x_ico[_X]);
  kico[_V] += learningRate_ico*deri_Penalty_actual*fabs(x_dot_com);//(x_ico[_V]);
  kico[TH] += learningRate_ico*deri_Penalty_actual*fabs(theta_com);//(x_ico[TH]);
  kico[OM] += learningRate_ico*deri_Penalty_actual*fabs(theta_dot_com);//(x_ico[OM]);
#endif

}




/************SIMULATION**************************************************************************/
#define FORCE_MAX 10.0
void cart_pole(int n, double t, double *x, double *xdot)
{

  double theta, theta_dot;
  double costheta, sintheta;
  double temp;
  double thetaacc, xacc;

  double sensorynoise;

  //sensorynoise = 0.001*uniform_dist();

  sensorynoise = 0.0;//0.000001*uniform_dist();

  theta = x[TH];//+sensorynoise;
  theta_dot = x[OM];

  costheta = cos(theta);
  sintheta = sin(theta);

  temp = (force+ POLEMASS_LENGTH * theta_dot * theta_dot * sintheta)/ TOTAL_MASS;

  thetaacc = (GRAVITY * sintheta - costheta* temp)
    		        / (LENGTH * (FOURTHIRDS - MASSPOLE * costheta * costheta/ TOTAL_MASS));

  xacc  = temp - POLEMASS_LENGTH * thetaacc* costheta / TOTAL_MASS;

  xdot[TH] = x[OM];
  xdot[_X] = x[_V];
  xdot[OM] = thetaacc;
  xdot[_V] = xacc;
}

/****
 *	Runge-Kutta
 ****/

void rkstep( void (*vf)(),	/* vector field vf( n, t, y, ydot) */
    int n, double *y,
    double *pt, double dt,
    double *work)	/* work area >= 7*n */
{
  double	*y2, *y3, *y4, *k1, *k2, *k3, *k4;
  double	dt2 = dt/2;
  int 	i;

  y2 = work;
  y3 = work + n;
  y4 = work + 2*n;
  k1 = work + 3*n;
  k2 = work + 4*n;
  k3 = work + 5*n;
  k4 = work + 6*n;
  //fprintf(fdata2,"%d %f %f %f %f %f %f %f %f %f\n",ntrial, xt[_X /*input 0*/],xt[_V /*input 1*/], xt[TH /*input 2*/],xt[OM /*input 3*/], output, k[0], k[1], k[2], k[3]);

  vf( n, *pt, y, k1);
  for( i = 0; i < n; i++){
    y2[ i] = y[ i] + k1[ i]*dt2;
  }
  *pt += dt2;
  vf( n, *pt, y2, k2);
  for( i = 0; i < n; i++){
    y3[ i] = y[ i] + k2[ i]*dt2;
  }
  vf( n, *pt, y3, k3);
  for( i = 0; i < n; i++){
    y4[ i] = y[ i] + k3[ i]*dt;
  }
  *pt += dt2;
  vf( n, *pt, y4, k4);
  for( i = 0; i < n; i++){
    y[ i] += ( k1[ i] + 2*k2[ i] + 2*k3[ i] + k4[ i])*dt/6;
  }
}
