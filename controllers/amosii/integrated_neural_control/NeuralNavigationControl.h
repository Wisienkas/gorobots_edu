/*
 * NeuralNavigationControl.h
 *
 *  Created on: 28.04.2014
 *      Author: Dennis Goldschmidt
 */

#ifndef NEURALNAVIGATIONCONTROL_H_
#define NEURALNAVIGATIONCONTROL_H_

#include <cmath>
#include <fstream>
#include <random>
#include <vector>
#include "selforg/matrixutils.h"
#include "selforg/matrix.h"
#include <ode_robots/amosiisensormotordefinition.h>
#include "navigation_modules/PathIntegrationMechanism.h"
using namespace std;

/*
 * Neural Navigation Control Class
 *
 */
class NeuralNavigationControl {

public:
	NeuralNavigationControl(bool navi_opt);
	~NeuralNavigationControl();
	double getNormalRandom(double mean, double std);
	double step_nnc(const vector<double> in_sensor, const vector< vector<double> > in_prepro);
	double steering_command; // output signal of controller
	double compass_input;
	double speed_input;

	//matrix::Matrix HDmat;

private:
	PathIntegration * pi;

	bool navi_on;
	bool pi_only;
	double rand_dir;
	const int turn_interval = 100;
	int global_count;

	ofstream nnc_data;
	ofstream hd_array;
};



#endif /* NEURALNAVIGATIONCONTROL_H_ */
