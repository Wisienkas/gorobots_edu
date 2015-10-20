/*
 * NeuralNavigationControl.h
 *
 *  Created on: 28.04.2014
 *      Author: Dennis Goldschmidt
 */

#ifndef NEURALNAVIGATIONCONTROL_H_
#define NEURALNAVIGATIONCONTROL_H_

#include <random>
#include <vector>
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
	double step_nnc(const vector<double> in_sensors, const vector< vector<double> > in_prepro);
	double steering_command; // output signal of controller


private:
	bool navi_on;
	bool pi_only;
	double rand_dir;
	const int turn_interval = 100;
	int global_count;
};



#endif /* NEURALNAVIGATIONCONTROL_H_ */
