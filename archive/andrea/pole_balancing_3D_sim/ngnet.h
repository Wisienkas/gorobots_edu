/*
 * ngnet.h
 *
 *  Created on: Jun 15, 2011
 *      Author: poramate
 */

#ifndef NGNET_H_
#define NGNET_H_



#define NONE  0
#define WRAP  1

#define IN    6 //= XDIM  //.e.g, 6 pre_initial BUT here use only 2 arrays = 2 inputs
#define OUT   1 //= UDIM //.e.g, 2 pre_initial BUT here use only 1 array  = 1 output
#define UNITNUM 5000 //pre_initial

typedef struct defunit{
	int ni,no;
	double cx[IN]; /* centers of basis*/
	double ac;
	double softac;
	double ivar[IN];
	double gx[IN];
	double w[OUT];  /* weight:w[no]*/ //------weights of critic
	double dw[OUT];
}Unit,*unit;

typedef struct defcell{
	int fstf;
	Unit cell[UNITNUM];
}Cell;


class NGNet{
public:

	//Initial RBF
	double active_thresh;
	void put_incsbox(Cell *isb, int ni, int no, double *xp, double *Ivr, int *nc); ///????
	int init_incsbox(Cell *isb,int ni, int no);
	int reset_incsbox(Cell *isb);

	//Calculate value function
	void incsbox_output(Cell *isb, double *x, double *y,int *nc);
	//Calculate neural activation with softmax
	double incsbox_activate( Cell *isb, double *x, int *nc);
	//Calculate neural activation without softmax
	void incsbox_unit(Unit *cunit, double *x);
	//Calculate update value function trace
	void incsbox_trace(Cell *isb, double *x, double lambda, int *nc);
	//Calculate update weight of value function--> Learning mechanism of critic
	double incsbox_update( Cell *isb, double *x, double *error, double rate, int *nc,
			double *Ivr, double Thr ,double near);

	////// Used function
//	extern double active_thresh;
//	extern int init_incsbox(Cell *isb,int ni, int no);
//	extern int reset_incsbox(Cell *isb);
//	extern void incsbox_output( Cell *isb, double *x, double *y,int *nc);
//	extern double incsbox_activate( Cell *isb, double *x,int *nc);
//	extern void incsbox_unit(Unit *cunit, double *x);
//	extern void incsbox_trace(Cell *isb, double *x, double lambda, int *nc);
//	extern double incsbox_update( Cell *isb, double *x, double *error, double rate, int *nc,
//			       double *Ivr, double Thr, double near);
	//////






//_________////// UNUsed function___________________________________________________________________//

//	extern void incsbox_predict( Cell *isb,double *y,double suma,int *nc); NOT exit!
//	extern int reset_incsbox_trace(Cell *isb, int *nc);
//	extern void incsbox_jacobian( Cell *isb, double *x,double **jac,int *nc);
//	extern double **realloc_matrix( double **ptr, int row, int column);
//	extern void incsbox_trace_elig(Cell *isb, double *x, double *coeff, double lambda, int *nc);

//	extern int save_center(FILE* fp, Cell* isb, int *unum);
//	extern int save_incsbox(FILE* fp, Cell* isb, int *unum);
//	extern int load_incsbox(FILE* fp, Cell* isb, int *unum);
//	extern double incsbox_target( Cell *isb, double *x, double *yt, double rate, int *nc,
//				      double *Ivr, double Thr, double near);
//	extern void plot_func(Cell *isb, int *nc, char *filename);
//	extern int save_incsbox(FILE* fp, Cell* isb,int *unum);
//	extern int load_incsbox(FILE* fp, Cell* isb, int *unum);



};

#endif /* NGNET_H_ */
