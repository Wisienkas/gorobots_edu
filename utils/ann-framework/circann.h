/*
 * circann.h
 *
 *  Created on: 21.10.2015
 *      Author: Dennis Goldschmidt
 */

#ifndef CIRCANN_H_
#define CIRCANN_H_

#include <cmath>
#include "ann.h"
using namespace std;

class CircANN : public ANN {
public:

    /**
     * The constructor.
     *
     * @param numneurons Number of neurons
     */
	CircANN(int numneurons);

    /**
     * The destructor.
     */
	~CircANN();

    /**
     * Returns the maximum firing rate in the circular array.
     *
     * @return (double)
     */
	double getMaxRate();

    /**
     * Returns the preferred orientation of a given neuron.
     *
     * @param (int) index: neuron index
     * @return (double)
     */
	double getPrefAngle(int index);

    /**
     * Returns the sum of firing rate in the circular array.
     *
     * @return (double)
     */
	double getSumRate();

    /**
     * Returns the angle of the population vector average in the circular array.
     *
     * @return (double)
     */
	double getVecAvgAngle();
};

#endif /* CIRCANN_H_ */
