/*
 * psn.cpp
 *
 *  Created on: Feb 29, 2012
 *      Author: timo
 */

#include "psn.h"

PSN::PSN()
{
    setNeuronNumber(12);

    // synaptic weights
    w( 2,  0, -5.0);
    w( 3,  1, -5.0);
    w( 4,  0, -5.0);
    w( 5,  1, -5.0);
    w( 6,  2,  0.5);
    w( 7,  3,  0.5);
    w( 8,  4,  0.5);
    w( 9,  5,  0.5);
    w(10,  6,  3.0);
    w(10,  7,  3.0);
    w(11,  8,  3.0);
    w(11,  9,  3.0);

    // neuron biases
    b( 0,  1.0 );
    b( 6,  0.5 );
    b( 7,  0.5 );
    b( 8,  0.5 );
    b( 9,  0.5 );
    b(10, -1.35);
    b(11, -1.35);
}
