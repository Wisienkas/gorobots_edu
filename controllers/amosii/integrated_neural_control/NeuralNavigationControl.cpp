/*
 * NeuralNavigationControl.cpp
 *
 *  Created on: 28.04.2014
 *      Author: Dennis Goldschmidt
 */

#include "NeuralNavigationControl.h"
using namespace std;


NeuralNavigationControl::NeuralNavigationControl(bool navi_opt){
	steering_command = 0;
	navi_on = navi_opt;
	pi_only = false;
	global_count = 0;
}

NeuralNavigationControl::~NeuralNavigationControl(){

}

double NeuralNavigationControl::getNormalRandom(double mean, double std){
	static random_device e { };
	static normal_distribution<double> d(mean, std);
	return d(e);
}

double NeuralNavigationControl::step_nnc(const vector<double> in_sensors, const vector< vector<double> > in_prepro){
	if(!navi_on)
		return steering_command;

	if(global_count%turn_interval == 0)
		rand_dir = getNormalRandom(0.0, 1.0);

	steering_command = rand_dir;

	global_count++;
	return steering_command;
}
