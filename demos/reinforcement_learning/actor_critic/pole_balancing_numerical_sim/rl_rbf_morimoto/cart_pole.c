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

void cart_pole(int n, double t, double *x, double *xdot); 
void rkstep( void (*vf)(),	
	     int n, double *y,
	     double *pt, double dt,
	     double *work);	

double cp_time = 0.0;
double dt = 0.01;
double rk_buffer[XDIM*N_RKBUFF];
int nloop = 1000000;

int xdim = XDIM;
double force=0.0;

int nbasis = 0;
int basis[XDIM];
double wbasis[XDIM];

//double k[XDIM] = {-8.8632,  -14.9521,  -90.9478,  -25.0398};
double k[XDIM] = {0.0,  0.0,  0.0,  0.0};

double xt_limit = 2.6;
double theta_limit = 12.0/180.*M_PI;

double xmin[XDIM];
double xmax[XDIM];
double basis_width[XDIM];

Cell VALUE;

#ifndef clip
#define	clip(x,l,u)	( ((x)<(l)) ? (l) : ((x)>(u)) ? (u) : (x))
#endif

void init_funcapprox()
{
  int i;
  int state_index[XDIM]; 
  double xc[XDIM];
  //  FILE *fp;

  init_incsbox(&VALUE, xdim,  1);
  reset_incsbox(&VALUE);

  basis[_X] = 3;
  basis[_V] = 3;
  basis[TH] = 6;
  basis[OM] = 3;

  xmax[_X] = xt_limit;
  xmin[_X] =-xt_limit;

  xmax[_V] = 26.0;
  xmin[_V] =-26.0;

  xmax[TH] = theta_limit;
  xmin[TH] =-theta_limit;

  xmax[OM] = 180.0/180.0*M_PI;
  xmin[OM] =-180.0/180.0*M_PI;


  //  fp = fopen("basis_center.dat","a");
  nbasis = 0;
  for(i=0;i<xdim;i++)
    {
      basis_width[i]  = (xmax[i] - xmin[i])/(2.0*basis[i]);
      wbasis[i] = 1.0/basis_width[i];
    }

  for(state_index[_X]=0; state_index[_X]<basis[_X]; state_index[_X]++) 
    {
      for(state_index[_V]=0; state_index[_V]<basis[_V]; state_index[_V]++) 
	{
	  for(state_index[TH]=0; state_index[TH]<basis[TH]; state_index[TH]++) 
	    {
	      for(state_index[OM]=0;state_index[OM]<basis[OM]; state_index[OM]++) 
		{
		  for(i=0;i<xdim;i++)
		    {
		      xc[i]  = (xmin[i]+basis_width[i])+state_index[i]*2.0*basis_width[i];
		      //		      fprintf(fp,"%lf ",xc[i]);
		    }
		  //		  fprintf(fp,"\n");
		  put_incsbox(&VALUE, xdim, 1, xc, wbasis, &nbasis);

		}
	    }
	}
    }

  //  fclose(fp);

}

int main(int argc, char **argv)
{
  xt[_X] = 0.0;
  xt[_V] = 0.0;
  xt[TH] = 0.2;
  xt[OM] = 0.0;

  srand( time( NULL));

  init_funcapprox();
  gl_init_anim(argc, argv);

  /* GLUT main loop  */
  glut_loop();

  return 0;
}

double sig_out[UDIM]={0.0};
double si = 10.0;
double exp_output[UDIM];

double unit_activation_thresh=0.6;;
double error_thresh=0.1; 
double	tau = 0.09;	/* value horizon */
double	tau_elig = 0.01;	/* time const for critic */
double	tau_elig_output = 0.05;	/* time const for critic */
double	kappa    = 0.2;	/* time const for actor  */
double  td_limit = 1.0;
double  sig_elig[UDIM];
double  output_grad[UDIM];
double  rate_valuefunction = 0.5;
double  rate_policy = 0.5;
double  gam = 0.95;

/****
*	Gaussian random variable: just a sum of uniform distribution
*	Average = 0, Variance = 1
****/

double gauss()
{
	double	sum;
	int 	i;

	for( sum = i = 0; i < 12; i++) sum += (1.0*(double)rand()/(RAND_MAX));
	return( sum - 6.0);
}

double output_elig[XDIM]={0.0, 0.0, 0.0, 0.0};
void exploration_grad(double *exp_output, double *sig_elig, double *sig, double si)
{
  int i;

  for(i=0;i<UDIM;i++)
    {
      sig_elig[i] = (exp_output[i]*exp_output[i] - sig[i]*sig[i])*(1 - (sig[i]/si))/(sig[i]*sig[i]);
      sig_elig[i] = clip(sig_elig[i], -1.0, 1.0);
    }
}

void exploration_output(double *sig, double si, double *x)
{
  int i;

  for(i=0;i<UDIM;i++)
    sig[i] = si/(1+exp(-sig_out[i]));
}

double sig[UDIM];
void output_policy(double *x, double *u)
{
  int i;
  double u_tmp = 0.0;

     
  for(i=0;i<XDIM;i++)
    {
      u_tmp += (-(k[i])*x[i]);
    }
  
  exploration_output(sig, si, x);
  exp_output[0]=sig[0]*gauss();
  u[0] = u_tmp+exp_output[0];

}

void update_policy(double td_error, double rate)
{
  int i;

  for(i=0;i<XDIM;i++)
    k[i] += rate*td_error*output_elig[i];

  sig_out[0] += rate*td_error*sig_elig[0];
}

void update_policy_trace(double *xt, double lambda)
{
  int i;

  for(i=0;i<XDIM;i++)
    {
      output_elig[i] *= lambda;
      output_elig[i] -= output_grad[0]*xt[i];
    }
}

double reward_function(double *x, double *u)
{
  double r=0.0;

  if(x[_X] < -xt_limit || x[_X] > xt_limit)
    r += -1.0;
  
  if(x[TH] < -theta_limit || x[TH] > theta_limit)
    r += -1.0;

  return r;
}

double value_function(double *x)
{
  double Value;

  incsbox_output(&VALUE, x, &Value, &nbasis);

  return Value;
}

void update_valuefunction( double *x, double td, double rate)
{
  double td_error_tmp[1], vt, yt[1];

  td_error_tmp[0] = -td;
  incsbox_update( &VALUE, x, td_error_tmp, rate, &nbasis, wbasis, error_thresh, unit_activation_thresh);
}

void update_valuefunction_trace(double *x)
{
  incsbox_trace(&VALUE, x, 0.0, &nbasis);
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

double uniform_dist()
{
  double value;

  value = 2*((double)rand()/(RAND_MAX))-1.0;

  return value;
}

void reset_state(double *x)
{
  int i;

  for(i=0;i<XDIM;i++)
    xt[i] = 0.0;

   xt[_X] = 0.5*xt_limit*uniform_dist();
   xt[TH] = 0.5*theta_limit*uniform_dist();
}

static double acum_reward = 0.0;
static double balance_time = 0.0;
#define MAX_ITER  2000 // 20 [sec]
#define MAX_TRIAL 500
double balance_time_log[MAX_TRIAL];
double sig_out_log[MAX_TRIAL];
static int failure_flag = 1; //<--- initialize
int ntrial = 0;

double  param_log[MAX_TRIAL][XDIM];

void simulation_loop()
{
  static int iter = 0;
  int i;
  double  rt, Vt;
  double  td_error = 0.0;
  static double Vt_old = 0.0;
  FILE *fp;
  int m;

  if(failure_flag || iter >= MAX_ITER)
    {
      reset_state(xt);
      failure_flag = 0;
      Vt_old = 0.0;
      acum_reward = 0.0;
      balance_time_log[ntrial] = balance_time;
      sig_out_log[ntrial] = sig_out[0];
      balance_time = 0.0;
      ntrial++;
      iter = 0;
    }

  if(ntrial < MAX_TRIAL)
   {
     /* Output from current policy */
     output_policy(xt, ut);
     force = 10.*ut[0];
     
     /* Cart Pole Dynamics */
     rkstep(&cart_pole, xdim, xt, &cp_time, dt, rk_buffer);
     /* Reward */
     rt = reward_function(xt,ut);

     balance_time += dt;

     /* Check state limit */
     failure_flag = check_limit(xt);

     /* Value function estimation for current state */
     Vt = value_function(xt);

     if(failure_flag)
       {
	 td_error = rt - Vt_old;
	 td_error = clip( td_error, -td_limit, td_limit);
       }
     else
       {
	 td_error = rt + gam*Vt - Vt_old;
	 td_error = clip( td_error, -td_limit, td_limit);
	 Vt_old = Vt;
       }

     /* gradients for policy parameters*/
     exploration_grad(exp_output, sig_elig, sig, si);
     for(i=0;i<UDIM;i++)
       output_grad[i] = clip(exp_output[i]/(sig[i]*sig[i]),-1.0, 1.0);

     /* Update parameters for value function approximator */
     update_valuefunction_trace(xt);
     update_valuefunction(xt, td_error, rate_valuefunction);

     /* Update parameters for policy */
     update_policy_trace(xt,0.0); 
     update_policy(td_error, rate_policy);

     iter++;
   }
  else
    {
      fp = fopen("balance_time.dat","w");
      for(i=0;i<MAX_TRIAL;i++)
	{
	  fprintf(fp,"%lf\n",balance_time_log[i]);
	}
      fclose(fp);

      fp = fopen("sig_out.dat","w");
      for(i=0;i<MAX_TRIAL;i++)
	{
	  fprintf(fp,"%lf\n",sig_out_log[i]);
	}
      fclose(fp);

      fp = fopen("param.dat","w");
      for(i=0;i<MAX_TRIAL;i++)
	{
	  for(m=0;m<XDIM;m++)
	    fprintf(fp,"%lf ",param_log[i][m]);
	  fprintf(fp,"\n");
	}
      fclose(fp);

      exit(0);
    }
}

#define FORCE_MAX 10.0
void cart_pole(int n, double t, double *x, double *xdot)
{

  double theta, theta_dot;
  double costheta, sintheta;
  double temp;
  double thetaacc, xacc;

  theta = x[TH];
  theta_dot = x[OM];
  
  costheta = cos(theta);
  sintheta = sin(theta);

  temp = (force + POLEMASS_LENGTH * theta_dot * theta_dot * sintheta)/ TOTAL_MASS;
  
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
