/*
 * combination.cpp
 *
 *  Created on: Feb 28, 2012
 *      Author: timo
 */

#include "combination.h"

#include "so2cpg.h"
#include "vrn.h"
#include <iostream>

Combination::Combination()
{
    // this net has no own neurons
    setNeuronNumber(0);

    // create the subnets and add them
    cpg      = new SO2CPG();
    vrnLeft  = new VRN();
    vrnRight = new VRN();
    addSubnet(cpg);
    addSubnet(vrnLeft);
    addSubnet(vrnRight);

    // create connections from CPG to VRNs
    w(vrnLeft->getNeuronX(),  cpg->getNeuron(0), 3);
    w(vrnRight->getNeuronX(), cpg->getNeuron(0), 3);

    // set static input to second input of VRN
    setInput(vrnLeft ->getNeuronY(), 1);
    setInput(vrnRight->getNeuronY(), -1);

    // set frequency of the network
    cpg->setPhi(0.2);
}

Combination::~Combination()
{
}

const double& Combination::getO1()
{
    return getOutput(vrnLeft->getNeuronOutput());
}

const double& Combination::getO2()
{
    return getOutput(vrnRight->getNeuronOutput());
}
