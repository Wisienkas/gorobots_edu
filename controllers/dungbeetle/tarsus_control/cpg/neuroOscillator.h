/*
 * neuronOscillator.h
 *
 *  Created on: Sep 27, 2014
 *      Author: giuliano
 */

#ifndef NEUROOSCILLATOR_H_
#define NEUROOSCILLATOR_H_

#include <stdio.h>
#include <math.h>
#include <vector>
#include <iostream>
#include <string>
#include <fstream>
#include <algorithm>


using namespace std;



class neuroOscillator {
private:
	double w00;
	double w11;
	double w01;
	double w10;
	double out0_t;
	double out1_t;
	double out0_t1;
	double out1_t1;
	double Phi,Alpha;
public:

	neuroOscillator(double out0_init,double out1_init,double alpha,double phi);
	virtual ~neuroOscillator();


	void printWeights();
	void computeOut0();
	void computeOut1();
	void update();
	double getFrequency();
	double getOutPut0();
	double getOutPut1();




};

#endif /* NEURONOSCILLATOR_H_ */
