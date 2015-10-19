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
	vector<double> step();
	double getNormalRandom(double mean, double std);

private:
	vector<double> navi_output;
	bool navi_on;
	bool pi_only;
	double rand_dir;
	const int turn_interval = 100;
	int global_count;
};



#endif /* NEURALNAVIGATIONCONTROL_H_ */
