/*
 * vrn.cpp
 *
 *  Created on: Feb 28, 2012
 *      Author: timo
 */

#include "vrn.h"

VRN::VRN()
{
    setNeuronNumber(7);

    w(2, 0, 1.7246);
    w(2, 1, 1.7246);
    w(3, 0, -1.7246);
    w(3, 1, -1.7246);
    w(4, 0,  1.7246);
    w(4, 1, -1.7246);
    w(5, 0, -1.7246);
    w(5, 1,  1.7246);
    w(6, 2,  0.5);
    w(6, 3,  0.5);
    w(6, 4, -0.5);
    w(6, 5, -0.5);

    b(2, -2.48285);
    b(3, -2.48285);
    b(4, -2.48285);
    b(5, -2.48285);
}

Neuron* VRN::getNeuronX()
{
    return getNeuron(0);
}

Neuron* VRN::getNeuronY()
{
    return getNeuron(1);
}

Neuron* VRN::getNeuronOutput()
{
    return getNeuron(6);
}
