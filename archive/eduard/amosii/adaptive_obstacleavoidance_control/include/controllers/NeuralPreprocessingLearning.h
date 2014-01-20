/*
 * NeuralPreprocessingLearning.h
 *
 *  Created on: May 2, 2011
 *      Author: poramate
 *
 *      Edited by Dennis Goldschmidt
 *      Apr 27, 2012
 *
 */

#ifndef NEURALPREPROCESSINGLEARNING_H_
#define NEURALPREPROCESSINGLEARNING_H_

#include <vector>
#include <cmath>
#include <ode_robots/amosiisensormotordefinition.h>
#include "delayline.h"
#include "utils/ann-framework/ann.h"

//Save files
#include <iostream>
#include <fstream>
#include <string.h>
using namespace std;
//Save files


//1) Classs for US Oabstacleavoidance------------

class US_Obstacleavoidance:  public ANN {

public:
	US_Obstacleavoidance();
	~US_Obstacleavoidance();

	void step_oa();
	double getOutput(int i);
	double weight_neuron1;
	double weight_neuron2;
	double weight_neuron3;
	double weight_neuron4;
	double neuron_weightsum_inhib;
	double i1_refl;
	double i2_refl;
	double reflex_cut;
	double e;
	double i1;
	double i2;
	double u1;
	double u2;
	double v1;
	double v2;
	double u3;
	double u4;
	double v3;
	double v4;

	double delta_w1;
	double delta_w2;

	double mu;
	double gamma;
	double vt;

	double mu2;
	double gamma2;
	double vt2;
	double gain;
	int mode;
	ofstream outTezinhib;
	ofstream outOAValues;
	int runsteps;
	int steps;
	bool debug;

};

//2) Class for Neural preprocessing and learning------------


class NeuralPreprocessingLearning{

public:

	//---Start Define functions---//
	NeuralPreprocessingLearning();
	~NeuralPreprocessingLearning();
	std::vector< vector<double> > step_npp(const std::vector<double> in0);

	double sigmoid(double num)
	{
		return 1.0/(1.0+exp(-num));
	}

	//---End Define functions---//

	//---Start Define vector----//
	// add public available variables
	std::vector<double> mappingsensor;
	std::vector<double> sensor_activity;
	std::vector<double> sensor_output;
	std::vector< vector<double> > preprosensor;
	//std::vector< vector<ANN*> > preproobjvect;
	std::vector<double> ir_predic_activity;
	std::vector<double> ir_reflex_activity;
	std::vector<double> ir_predic_output;
	std::vector<double> ir_reflex_output;
	std::vector<double> ir_predic_w;
	std::vector<double> ir_reflex_w;
	std::vector<double> d_reflex_output;
	std::vector<double> drho1;
	std::vector<double> rho1;
	std::vector<double> irlearn_output;
	std::vector<double> irlearn_output_prolong;
	std::vector<double> test_output;
	std::vector<int> counter_fs;
	std::vector<int> dcounter_fs;
	bool switchon_IRlearning;
	//---End Define vector----//

	//Delayline constructor
	std::vector< Delayline* > us_delayline;
	std::vector< Delayline* > irs_delayline;


	US_Obstacleavoidance* OA;

	//Save files
	ofstream outFilenpp1;

	bool FRONT_IR;
	double rate;

	//Save files

private:
	// add private  variables
	double sensor_w_pfs_rfs;
	double sensor_w_pfs_pfs;
	double irsensor_w_pfs_rfs;
	double irsensor_w_pfs_pfs;
	double lowpass;
	double threshold;
	double ir_learnrate;
	int delay;
	int delay_irs;



};



#endif /* NEURALPREPROCESSINGLEARNING_H_ */

