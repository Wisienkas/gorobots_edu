/***********************************************/
/*                                             */
/*  NGnet.c                                    */
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
#include "NGnet.h"


#if 0
typedef struct defunit{
          int ni,no;
          double cx[IN]; /* centers of basis*/
          double ac;
          double softac;
          double ivar[IN];
          double gx[IN];
          double w[OUT];  /* weight:w[no]*/
	  double dw[OUT];
	}Unit,*unit;

typedef struct defcell{
    int fstf;
    Unit cell[UNITNUM];
}Cell;
#endif

double active_thresh = 0.001;

//Make box system with center and invarience
void put_incsbox(Cell *isb, int ni /*number input = e.g., 4*/, int no /*number output = e.g., 1*/, double *xp /*input vector*/, double *Ivr /*wbasis vector = width of each input*/, int *nc /*neuron number*/)
{
    int i,j;
    Unit *curr;

    curr = isb->cell+(*nc);

    curr->ni = ni;
    curr->no = no;

	//printf("isb %d \n\n", isb->cell+(*nc));


    for(i=0;i<ni /*4 inputs*/;i++)
	{
	    curr->cx[i] = xp[i];
	    curr->gx[i] = 0.0;
	    curr->ivar[i] = Ivr[i];

	 // printf("cx[%d] = %f \n\n",i, curr->cx[i]);
	 // printf("ivar[%d] = %f \n\n",i, curr->ivar[i]);
	}

    for(i=0;i<no /*1 output*/;i++)
	curr->w[i] = 0.0;

    (*nc)++; //nc= 162 neurons

}
 
//-----------------------------------------------------
int init_incsbox(Cell *isb,int ni, int no)
{
  int i,j;
  Unit *curr;

  curr = isb->cell;

  curr->ni = ni;
  curr->no = no;
  printf("#output:%d\n",curr->no);

  return(1);
}

int reset_incsbox_trace(Cell *isb, int *nc)//????????????????????????????????????????HELP
{
 Unit *curr; 
 int i,unum,nin,nout;

 curr = isb->cell;
 nin = curr->ni;
 nout = curr->no;

 for(unum=0;unum<*nc;unum++,curr++)
   {
     for(i=0;i<nin;i++)
       curr->gx[i]=0.0;

     curr->ac = 0.0;
     curr->softac = 0.0;

     for(i=0;i<nout;i++)
       curr->dw[i] = 0.0;
    }

 return(1);
}

int reset_incsbox(Cell *isb)
{
 Unit *curr; 
 int i,unum,nin,nout;

 isb->fstf = 1;

 curr = isb->cell;
 nin = curr->ni;
 nout = curr->no;

 for(i=0;i<nin;i++)
    {
      curr->cx[i]=0.0;
      curr->gx[i]=0.0;
      curr->ivar[i] = 1.0;
    }

 for(i=0;i<nout;i++)
      curr->w[i]=0.0;

  curr->ac = 0.0;
  curr->softac = 0.0;

 for(i=0;i<nout;i++)
   curr->dw[i] = 0.0;
  
 return(1);
}

double incsbox_target( Cell *isb, double *x, double *yt, double rate, int *nc,
		       double *Ivr, double Thr, double near)
{
  double y[OUT],err[OUT],ui[IN];
  int nin,nout,k,i,j,unum,errf;
  double serr,ac,softac,msqr,maxac=0.0,sume=0.0,ca;
  Unit *curr;
  double dJ;
  double dJ_dc;
  double ivar_up,cx_up,ivar_max,cx_max;



  curr = isb->cell;

  nin = curr->ni;
  nout = curr->no;

  if(isb->fstf)
      {
	printf("FIRST ac:%lf\n",Ivr[0]);
	  for(i=0;i<nin;i++)
	      {
		  curr->cx[i]=x[i];
		  curr->ivar[i] = Ivr[i];
	      }

	  for(i=0;i<nout;i++)
	      curr->w[i]=0.0; /*yt[i];*/
      }

  isb->fstf = 0;

  incsbox_output(isb,x,y,nc);

  serr = 0.0;
  for(i=0;i<nout;i++)
      {
	  err[i] = (y[i] - yt[i]);
	  serr+=err[i]*err[i];
      }

  /*printf("err:%lf\n",err[0]);*/

  for(i=0;i<nin;i++)
    ui[i] = x[i] -  curr->cx[i];

  for(unum=0;unum<*nc;unum++,curr++)
      {
	  int i,j;


	  if(curr->softac == 0.0) continue;

	  ac = curr->ac;

	  softac = curr->softac;
	  if(ac > maxac)	  maxac=ac;

	  sume = 0.0;
	  for(k=0;k<nout;k++){
	      sume += err[k]*curr->w[k];
	      curr->w[k]-=rate*err[k]*softac;         /*update weight*/

	      //	      printf("weight:%lf\n",curr->w[k]);
	  }
      }


 msqr = sqrt(serr/nout);   /* mean squared error*/



  /*create new basis function*///--------------------------------------------------------------ADAPTIVE CRITIC with new basis elements


 if(msqr > Thr && maxac < near && (*nc) < 500)
     {
	 int i,j;
         Unit *ptr;


	 ptr=isb->cell+(*nc);

         ptr->ni=nin;
         ptr->no=nout;

	 for(i=0;i<nin;i++)
	     {
		 ptr->cx[i] = x[i];
		 ptr->gx[i] = 0.0;
		 ptr->ivar[i] = Ivr[i];
	     }

	 for(i=0;i<nout;i++)
	     ptr->w[i] =  0.0; /*yt[i];*/

	 ptr->softac = 0.0; /*check!*/
	 ptr->ac = 0.0;

	 for(i=0;i<nout;i++)
	   ptr->dw[i] = 0.0;

         (*nc)++;

	 printf("new basis: %d\n",(*nc));

     }

 return( msqr);
}

/*******
*  Eligibility Trace //////////////////ADD
********/

void incsbox_trace(Cell *isb, double *x, double lambda, int *nc)
{
  int k;
  int nout;
  int unum;
  double y[OUT];
  Unit *curr;

  incsbox_output(isb,x,y,nc);

  curr=isb->cell;

  nout = curr->no;

  for(unum=0;unum<*nc;unum++,curr++)
    {
      for(k=0;k<nout /*1-> one output actor*/;k++)
	{
	  curr->dw[k] *= lambda;
	  curr->dw[k] += (1-lambda)*(curr->softac);
	}
     // printf("nc %d \n", *nc);
    }

}


//////////////////ADD
void incsbox_trace_elig(Cell *isb, double *x, double *coeff, double lambda, int *nc)
{
  int k;
  int nout;
  int unum;
  double y[OUT];
  Unit *curr;

  incsbox_output(isb,x,y,nc); // neural activation!!!!

  curr=isb->cell;

  nout = curr->no;

  for(unum=0;unum<*nc;unum++,curr++)
  {
	  for(k=0;k<nout;k++)
	  {
		  curr->dw[k] *= lambda;
		  //	  curr->dw[k] += (1-lambda)*(coeff[k])*(curr->softac);
		  curr->dw[k] += (coeff[k])*(curr->softac); // hidden neural activation softmax!!!!
	  }
  }
}

//////////////////ADD
double incsbox_update( Cell *isb, double *x, double *error, double rate, int *nc,
		       double *Ivr, double Thr ,double near)
{
  double y[OUT],err[OUT],ui[IN];
  int nin,nout,k,i,j,unum,errf;
  double serr,ac,msqr,maxac=0.0,sume=0.0,ca;
  Unit *curr;
  double dJ;
  double dJ_dc;
  double ivar_up,cx_up,ivar_max,cx_max;

  curr = isb->cell;

  nin = curr->ni;
  nout = curr->no;

  if(isb->fstf)
      {
	  for(i=0;i<nin;i++)
	      {
		  curr->cx[i]=x[i];
		  curr->ivar[i] = Ivr[i];
	      }

	  for(i=0;i<nout;i++)
	      curr->w[i]=0.0;
      }

  isb->fstf = 0;

  incsbox_output(isb,x,y,nc);

  /*printf("error:%lf\n",error[0]);*/

  for(i=0;i<nin;i++)
    ui[i] = x[i] -  curr->cx[i]; 

  for(unum=0;unum<*nc;unum++,curr++)
      {
	ac = curr->ac;
	if(ac > maxac)	  maxac=ac;
	  
	sume = 0.0;
	for(k=0;k<nout;k++){
	  sume += error[k]*curr->w[k];
	  curr->w[k]-=rate*error[k]*(curr->dw[k]); /*update weight*/
	  }
      }

  serr = 0.0;
  for(i=0;i<nout;i++)
    serr += error[i]*error[i];

 msqr = sqrt(serr/nout);   /* mean squared error*/

  /*create new basis function*/

 if(msqr > Thr && maxac < near && (*nc) < Max_RBF_neuron)
     {
	 int i,j; 
         Unit *ptr;

	 ptr=isb->cell+(*nc);

         ptr->ni=nin;
         ptr->no=nout;

	 for(i=0;i<nin;i++)
	     {
		 ptr->cx[i] = x[i];
		 ptr->gx[i] = 0.0;
		 ptr->ivar[i] = Ivr[i];
	     }

	 for(i=0;i<nout;i++)
	   {
	     ptr->w[i]  = 0.0 ; /*y[i] - error[i];*/
	     ptr->dw[i] = 0.0;
	   }

	 ptr->softac = 0.0; /*check!*/
	 ptr->ac = 0.0;

         (*nc)++;

	 printf("new basis: %d\n",(*nc));
     }

 return( msqr);
}

/////CALL to calculate V --1)
void incsbox_output( Cell *isb, double *x, double *y,int *nc)
{
  //double suma;
  Unit *curr;
  int i,unum;

  incsbox_activate( isb, x, nc /*e.g., 162 neurons*/); // Return activation of neurons!

  curr = isb->cell;

  /*for(i=0;i<curr->no;i++)
      if(y != NULL)y[i]=0.0; // Reset all to zero first before summing!
   */

  for(i=0;i<curr->no /*no = 1*/;i++)
  {
	  if(y != NULL)
	  {
		  y[i]=0.0; // Reset all to 1 output to zero first before summing!
	  }

  }

  for(unum=0;unum<*nc /*e.g., hidden neurons = 162 neurons*/;unum++,curr++)
  {
	 // printf("Before %d \n",unum);
	 // printf("before %f\n", curr->softac);

	  if(curr->softac == 0.0) continue; // If not == 0 then do below but if == 0 then out of loop

	  for(i=0;i<curr->no;i++) {
		  y[i]+=(curr->w[i])*(curr->softac /*return from incsbox_activate()*/);

		  // printf("um %d  i %d cu %f y = %f curr->softac %f\n",unum, i,curr->w[0], y[0], curr->softac);
	  }

	  //printf("%d --> y[0] = %f\n",unum, y[0]); //-------------------------------------------------------------------------PRINT to debug ASK morimoto
  }

  //printf("Last line of incsbox_output() FINISH\n"); //-------------------------------------------------------------------------PRINT to debug morimoto

}
///CALL to calculate V --2)
double incsbox_activate( Cell *isb, double *x, int *nc)
{
	int i,unum,nin;
	double suma=0.0;
	double sumg[IN];
	Unit *curr;

	curr=isb->cell;
	nin = curr->ni/*number input = e.g., 4*/;

	for(i=0;i<nin /*number input = e.g., 4*/;i++)
		sumg[i]=0.0;

	for(unum=0;unum<*nc /*e.g., 162 neurons*/ ;unum++,curr++)
	{
		incsbox_unit(curr,x); //calculate activation of each neuron!e.g., 1,..,162 neurons --> return ac = neuron activation

		suma+=curr->ac; // sum all ac for softmax operation

		////////////////////////////////////////////////////////////////////????????////////////////////////////////////////
		for(i=0;i<nin;i++)
			sumg[i]+=curr->gx[i]*curr->ac;
		////////////////////////////////////////////////////////////////////????????////////////////////////////////////////

	}

	///Softmax operation!!!
	curr=isb->cell;
	for(unum=0;unum<*nc;unum++,curr++)
	{
		if(suma == 0.0)
		{
			curr->softac = curr->ac;
			for(i=0;i<nin;i++)
				curr->gx[i]=0.0;
		}
		else
		{
			curr->softac = (curr->ac)/suma;
			////////////////////////////////////////////////////////////////////????????////////////////////////////////////////
			if(curr->softac < active_thresh) curr->softac = 0.0;
			for(i=0;i<nin;i++)
				curr->gx[i]=curr->gx[i]-sumg[i]/suma;
			////////////////////////////////////////////////////////////////////????????////////////////////////////////////////
		}
	}
	/*return(suma); */
}

/*************************************************************
 *	Neuron activation of RBF
 *************************************************************/
void incsbox_unit(Unit *cunit, double *x) //--3)
{
  double ac=0.0; 
  double ui[IN];

  int i,j,k,nin;

  nin = cunit->ni;

  for(i=0;i<nin;i++)
      ui[i] = x[i] -  cunit->cx[i];

  //#ifdef PEND
  //  ui[0] = circ(ui[0],-M_PI,M_PI);
  //#endif

  for(i=0;i<nin;i++)
      {
	  ac += ui[i]*ui[i]*(cunit->ivar[i])*(cunit->ivar[i]);

	  ////////////////////////////////////////////////////////////////////????????////////////////////////////////////////
	  cunit->gx[i] = -ui[i]*(cunit->ivar[i])*(cunit->ivar[i]);
	  ////////////////////////////////////////////////////////////////////????????////////////////////////////////////////

      }

  cunit->ac = exp(-ac/2.0);

  // printf("ac out %d \n", nin);

}

void incsbox_jacobian( Cell *isb, double *x,double **jac, int *nc)
{
  int k,i,unum;
  double yc;
  Unit *curr;

  curr = isb->cell;
  incsbox_activate(isb,x,nc);

  for(k=0;k<curr->no;k++){
      for(i=0;i<curr->ni;i++){
	  jac[k][i]=0.0;}}

  for(unum=0;unum< *nc;unum++,curr++)
      {
	  for(k=0;k<curr->no;k++){
	      yc = (curr->w[k])*(curr->softac);
              for(i=0;i<curr->ni;i++)
		jac[k][i] += yc * (curr->gx[i]);
	  }
      }
}








