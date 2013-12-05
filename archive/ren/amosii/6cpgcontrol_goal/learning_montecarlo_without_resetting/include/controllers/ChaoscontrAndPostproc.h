/*
 * ChaoscontrAndPostProc.h
 *
 *  Created on: Aug 15, 2011
 *      Author: Ren Guanjiao
 */
 
#ifndef CHAOSCONTRANDPOSTPROC_H_
#define CHAOSCONTRANDPOSTPROC_H_


#include <vector>
#include <cmath>
//#include "sensor_motor_definition.h"

#include <ode_robots/amosiisensormotordefinition.h>

#include <assert.h>
#include <stdlib.h>

//Save files
#include <iostream>
#include <fstream>
#include <string.h>
using namespace std;
//Save files

#define PERIODMAX 10
#define FAC 8
#define	EPSILON 0.5

// Class for ChaosControl and PostProcessing
class ChaoscontrAndPostproc
{
public:
	ChaoscontrAndPostproc(int p1,int p2,int p3,int p4,int p5,int p6);
	~ChaoscontrAndPostproc();
	void ChaosControl();
	
	void reset();
	void reset_cli();
	void reset_cli1();
	void reset_cli2();
	void reset_cli3();
	void reset_cli4();
	void setPeriod(int period) { period_ = period;reset();};
	void setPeriod_cli(int period) { period_cli = period;reset_cli();};
	void setPeriod_cli1(int period) { period_cli1 = period;reset_cli1();};
	void setPeriod_cli2(int period) { period_cli2 = period;reset_cli2();};
	void setPeriod_cli3(int period) { period_cli3 = period;reset_cli3();};
	void setPeriod_cli4(int period) { period_cli4 = period;reset_cli4();};
	
	double getOutput1() {return output1;};
	double getOutput2() {return output2;};
	double getOutput1_cli() {return output1_cli;};
	double getOutput2_cli() {return output2_cli;};
	double getOutput1_cli1() {return output1_cli1;};
	double getOutput2_cli1() {return output2_cli1;};
	double getOutput1_cli2() {return output1_cli2;};
	double getOutput2_cli2() {return output2_cli2;};
	double getOutput1_cli3() {return output1_cli3;};
	double getOutput2_cli3() {return output2_cli3;};
	double getOutput1_cli4() {return output1_cli4;};
	double getOutput2_cli4() {return output2_cli4;};
	
	//The output before postprocessing
	/*double getOutput1() {return o1_;};
	double getOutput2() {return o2_;};
	double getOutput1_cli() {return o1_cli;};
	double getOutput2_cli() {return o2_cli;};
	double getOutput1_cli1() {return o1_cli1;};
	double getOutput2_cli1() {return o2_cli1;};*/
	
	double memory;
	double memory1;	
	double memory2;
	double memory3;
	double memory4;	
	bool synflag;
	bool synflag1;
	bool synflag2;
	bool synflag3;
	bool synflag4;

	ofstream fout; //to output the value to a txt file

	std::vector<double> step_nlm(const std::vector<double> in0);

protected:

	double w11_,w12_,w21_,w22_,theta1_,theta2_;
	double thetahys_[2];
	int Freq;
	double whys;
	double inverse_;
	
	//parameter of master
	double lr_;
	double o1_, o2_;
	int period_;
	int n_;
	double diff_n1_, diff_n2_, diffi_;
	double cl_;
	double input1_,input2_;
	bool learning;	
	int normalize_;
	int percount;
	double input_hys1,input_hys2;
	double ahys1,ahys2;
	double ahys_old1,ahys_old2;
	double slopeUP_;
	double slopeDOWN_;
	double output1;
	double output2;
	
	//parameter of client0
	double lr_cli;
	double o1_cli, o2_cli;
	int period_cli;
	int n_cli;
	double diff_n1_cli, diff_n2_cli, diffi_cli;
	double cl_cli;
	double input1_cli,input2_cli;
	bool learning_cli;
	int normalize_cli;
	int percount_cli;
	double input_hys1_cli,input_hys2_cli;
	double ahys1_cli,ahys2_cli;
	double ahys_old1_cli,ahys_old2_cli;
	double slopeUP_cli;
	double slopeDOWN_cli;
	double output1_cli;
	double output2_cli;
	double alpha_syn;
	
	//parameter of client1
	double lr_cli1;
	double o1_cli1, o2_cli1;
	int period_cli1;
	int n_cli1;
	double diff_n1_cli1, diff_n2_cli1, diffi_cli1;
	double cl_cli1;
	double input1_cli1,input2_cli1;
	bool learning_cli1;
	int normalize_cli1;
	int percount_cli1;
	double input_hys1_cli1,input_hys2_cli1;
	double ahys1_cli1,ahys2_cli1;
	double ahys_old1_cli1,ahys_old2_cli1;
	double slopeUP_cli1;
	double slopeDOWN_cli1;
	double output1_cli1;
	double output2_cli1;
	double alpha_syn1;
	
	//parameter of client2
	double lr_cli2;
	double o1_cli2, o2_cli2;
	int period_cli2;
	int n_cli2;
	double diff_n1_cli2, diff_n2_cli2, diffi_cli2;
	double cl_cli2;
	double input1_cli2,input2_cli2;
	bool learning_cli2;
	int normalize_cli2;
	int percount_cli2;
	double input_hys1_cli2,input_hys2_cli2;
	double ahys1_cli2,ahys2_cli2;
	double ahys_old1_cli2,ahys_old2_cli2;
	double slopeUP_cli2;
	double slopeDOWN_cli2;
	double output1_cli2;
	double output2_cli2;
	double alpha_syn2;
	
	//parameter of client3
	double lr_cli3;
	double o1_cli3, o2_cli3;
	int period_cli3;
	int n_cli3;
	double diff_n1_cli3, diff_n2_cli3, diffi_cli3;
	double cl_cli3;
	double input1_cli3,input2_cli3;
	bool learning_cli3;
	int normalize_cli3;
	int percount_cli3;
	double input_hys1_cli3,input_hys2_cli3;
	double ahys1_cli3,ahys2_cli3;
	double ahys_old1_cli3,ahys_old2_cli3;
	double slopeUP_cli3;
	double slopeDOWN_cli3;
	double output1_cli3;
	double output2_cli3;
	double alpha_syn3;
	
	//parameter of client4
	double lr_cli4;
	double o1_cli4, o2_cli4;
	int period_cli4;
	int n_cli4;
	double diff_n1_cli4, diff_n2_cli4, diffi_cli4;
	double cl_cli4;
	double input1_cli4,input2_cli4;
	bool learning_cli4;
	int normalize_cli4;
	int percount_cli4;
	double input_hys1_cli4,input_hys2_cli4;
	double ahys1_cli4,ahys2_cli4;
	double ahys_old1_cli4,ahys_old2_cli4;
	double slopeUP_cli4;
	double slopeDOWN_cli4;
	double output1_cli4;
	double output2_cli4;
	double alpha_syn4;

	typedef struct
	{
		double o1, o2;
	} data_t;
	data_t* data_;
	data_t* data_cli;
	data_t* data_cli1;
	data_t* data_cli2;
	data_t* data_cli3;
	data_t* data_cli4;
	
	double sigmoid_(double x) 
	{return (1./(1.+exp(-x)));};

	double tanh_(double x)
	{return (2./(1.+exp(-10.*x))-1);};
	
	double sigmoid_steep(double x)
	{return 1./(1.+exp(-1000*(10000-x)));};
	
	double deri_sigmoid_steep(double x)
	{
		double  deribeta_steep = 10;
		double  deritheta_steep = 10000;
		//deri sig = f(x)(1-f(x))
  	  	return (1./(1.+exp(-(deribeta_steep)*(deritheta_steep-x))))*(1-(1./(1.+exp(-(deribeta_steep)*(deritheta_steep-x)))));
	};

	void oscistep();
	void oscistep_cli();
	void oscistep_cli1();
	void oscistep_cli2();
	void oscistep_cli3();
	void oscistep_cli4();
	void step();
	void step_cli();
	void step_cli1();
	void step_cli2();
	void step_cli3();
	void step_cli4();
	void PostProcessing();
	void PostProcessing_cli();
	void PostProcessing_cli1();
	void PostProcessing_cli2();
	void PostProcessing_cli3();
	void PostProcessing_cli4();
	
};

#endif //CHAOSCONTRANDPOSTPROC_H_
