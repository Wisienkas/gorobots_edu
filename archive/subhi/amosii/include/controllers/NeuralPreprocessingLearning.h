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
#include "../utils/ann-framework/ann.h"

//Save files
#include <iostream>
#include <fstream>
#include <string.h>
using namespace std;
//Save files


//1) Classs for Preproobjects------------

class US_Obstacleavoidance: public ANN {

public:
	US_Obstacleavoidance();
	double getOutput(int i);

private:
	vector<double> output;
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
	std::vector< vector<ANN*> > preproobjvect;
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


	//Save files
	ofstream outFilenpp1;
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

