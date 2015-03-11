/*
  * plastic.h
 *
 *  Created on: Feb 5, 2015
 *      Author: giuliano
 */

#ifndef PLASTIC_H_
#define PLASTIC_H_

#include <vector>
#include <stdio.h>
#include <math.h>
#include <iostream>


class plastic {
private:
	double w2p_t,w20_t,w02_t,w2p_t1,w20_t1,w02_t1; // plasticity
	double w00,w01,w10,w11; // weights
	double A20,A02,B20,B02,B2p,A2p; // parameters
	double out0_t,out1_t,out2_t,out0_t1,out1_t1,out2_t1; // outputs
	double phi, alpha,learning; // parameters

public:
	plastic(double o0,double o1,double o2, double initial_phi,double _alpha);
	plastic(double o0,double o1,double o2, double initial_phi,double _alpha, double _bi);
	void update(double perturbation);
	void printOutputs();
	void updateWeights();
	double getOut0();
	double getOut1();
	double getOut2();
	double getFrequency();
	double getW20();
	double getW02();
	double getW2p();
	virtual ~plastic();
};

#endif /* PLASTIC_H_ */
