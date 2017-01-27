#ifndef _CPG_H
#define _CPG_H

#include <vector>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include "utils/ann-framework/ann.h"


typedef double parameter;


//Plasticity weights initial value must be initilialized 
extern  double EPSILON_INIT = 0.08;
extern  double BETA_INIT = 0;
extern  double GAMMA_INIT = 1;
//Parameters value
extern  parameter _A = 1.00;
extern  parameter _B = 0.01;
extern  parameter ALPHA = 1.01;
extern  parameter PHI_INIT = 0.04*2*3.14;
extern  parameter m = 1.0;


class CPG :public ANN
{

	private:
	//Perturbation neuron
		Neuron * P;
		double phi;
		parameter A_e,A_b,A_g;
		parameter B_e,B_b,B_g;

		
	public:

	//constructor
		CPG(double o0,double o1,double o2);
	//destructor
		~CPG(){}
	//update SO2 weights
		void updateWeights();
	//perturbate the oscillator
		void inputPerturbation(double P);
	//get output neuron 0
		double getOut0();
	//get output neuron 1
		double getOut1();
	//get output neuron 2
		double getOut2();
	//get oscillator frequency
		parameter getFrequency();
	//get Phi
		double getPhi();
	//get Epsilon
		double getEpsilon();
	//get Beta
		double getBeta();
	//get Gamma
		double getGamma();
	//get perturbation neuron output
		double getP();	
	//set phi
		void setPhi(double aphi);
	//set Epsilon
		void setEpsilon(double e);
	//set Beta
		void setBeta(double b);
	//set Gamma
		void setGamma(double g);
	
};
#endif
