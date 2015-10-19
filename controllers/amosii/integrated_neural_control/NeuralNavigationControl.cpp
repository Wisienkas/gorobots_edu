/*
 * NeuralNavigationControl.cpp
 *
 *  Created on: 28.04.2014
 *      Author: Dennis Goldschmidt
 */

#include "NeuralNavigationControl.h"
using namespace std;


NeuralNavigationControl::NeuralNavigationControl(bool navi_opt){
	navi_output.resize(1);
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

vector<double> NeuralNavigationControl::step(){
	if(!navi_on)
		return navi_output;

	if(global_count%turn_interval == 0)
		rand_dir = getNormalRandom(0.0, 1.0);

	navi_output.at(0) = rand_dir;

	global_count++;
	return navi_output;
}
