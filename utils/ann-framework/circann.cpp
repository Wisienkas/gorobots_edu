/*
 * circann.cpp
 *
 *  Created on: 21.10.2015
 *      Author: Dennis Goldschmidt
 */

#include "circann.h"
using namespace std;

CircANN::CircANN(int numneurons) : ANN(numneurons) {

}

CircANN::~CircANN() {

}

double CircANN::getMaxRate() {
	double max_rate = 0;
	for(unsigned int index = 0; index < N(); index++){
		double currOutput = getOutput(index);
		if(currOutput > max_rate)
			max_rate = currOutput;
	}
	return max_rate;
}

double CircANN::getPrefAngle(int index){
	return (2*M_PI*index)/N();
}

double CircANN::getSumRate(){
	double sum = 0;
	for(unsigned int index = 0; index < N(); index++)
		sum += getOutput(index);
	return sum;
}

double CircANN::getVecAvgAngle(){
	double sumx = 0;
	double sumy = 0;
	for(unsigned int index = 0; index < N(); index++){
		sumx += getOutput(index) * cos(getPrefAngle(index));
		sumy += getOutput(index) * sin(getPrefAngle(index));
	}
	return atan2(sumy, sumx);
}
