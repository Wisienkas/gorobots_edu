/*
 * sigmoid.h
 *
 *  Created on: May 28, 2013
 *      Author: ilyas
 */

#ifndef SIGMOID_H_
#define SIGMOID_H_

#include <math.h>


inline double sigmoid(int value){
	double s;
	s=1./(1.+exp(-value));
	return s;
};

inline double sigmoid_double(double value){
	double s;
	s=1./(1.+exp(-value));
	return s;
};


inline double sigmoid_inverted(int value){
	return sigmoid((-1.)*value);
};

#endif /* SIGMOID_H_ */
