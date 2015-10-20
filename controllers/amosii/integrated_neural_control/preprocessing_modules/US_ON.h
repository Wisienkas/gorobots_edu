/*
 * US_ON.h
 *
 *  Created on: Mar 25, 2015
 *      Author: degoldschmidt
 */

#ifndef US_ON_H_
#define US_ON_H_

#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <ode_robots/amosiisensormotordefinition.h>
#include "utils/ann-framework/ann.h"
#include "utils/ico-framework/ico.h"
using namespace std;

enum{reflex_on, predictive_on};


//1) Class for US Obstacle Negotiation Behavior ------------

class US_Obstaclenegotiation:  public ANN {
public:
	US_Obstaclenegotiation();
	~US_Obstaclenegotiation();

	void step();

private:
	ICO * learning;
	const double learn_rate = 0.01;
	const int num_signals = 2;
	const int delay = 100;
	const double lowpass = 0.01;
	bool learning_on;

	double reflex_input;
	double predictive_input;
};

#endif /* US_ON_H_ */
