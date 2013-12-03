/**************
 *       incsbox.h
 *    INcrimental SoftBOX representation of n-dim space
 *    March 1997, J. Morimoto
 ********/

#define NONE  0
#define WRAP  1

#define IN    6
#define OUT   2
#define UNITNUM 5000

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

extern double active_thresh;
extern int init_incsbox(Cell *isb,int ni, int no);
extern int reset_incsbox(Cell *isb);
extern int reset_incsbox_trace(Cell *isb, int *nc);
extern void incsbox_output( Cell *isb, double *x, double *y,int *nc);
extern void incsbox_predict( Cell *isb,double *y,double suma,int *nc);
extern double incsbox_activate( Cell *isb, double *x,int *nc);
extern void incsbox_unit(Unit *cunit, double *x);
extern void incsbox_jacobian( Cell *isb, double *x,double **jac,int *nc);
extern double **realloc_matrix( double **ptr, int row, int column);
extern void incsbox_trace(Cell *isb, double *x, double lambda, int *nc);
extern void incsbox_trace_elig(Cell *isb, double *x, double *coeff, double lambda, int *nc);
extern double incsbox_update( Cell *isb, double *x, double *error, double rate, int *nc,
		       double *Ivr, double Thr, double near);
extern int save_center(FILE* fp, Cell* isb, int *unum);
extern int save_incsbox(FILE* fp, Cell* isb, int *unum);
extern int load_incsbox(FILE* fp, Cell* isb, int *unum);
extern double incsbox_target( Cell *isb, double *x, double *yt, double rate, int *nc,
			      double *Ivr, double Thr, double near);
extern void plot_func(Cell *isb, int *nc, char *filename);
extern int save_incsbox(FILE* fp, Cell* isb,int *unum);
extern int load_incsbox(FILE* fp, Cell* isb, int *unum);

