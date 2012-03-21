/*
 * neuron.cpp
 *
 *  Created on: Feb 28, 2012
 *      Author: timo
 */

#include "neuron.h"

#include "ann.h"
#include "synapse.h"
#include "transferfunction.h"
#include <algorithm>
#include <iostream>

// initialize static constant member
const TanhFunction Neuron::defaultTransferFunction = TanhFunction();

Neuron::Neuron()
{
    activity = 0;
    output   = 0;
    bias     = 0;
    input    = 0;
    func     = &defaultTransferFunction;
}

Neuron::~Neuron()
{
    while (synapsesIn.size()>0)  delete synapsesIn.front();
    while (synapsesOut.size()>0) delete synapsesOut.front();
}

void Neuron::addSynapseIn(Synapse * synapse)
{
    synapsesIn.push_back(synapse);
}

void Neuron::addSynapseOut(Synapse * synapse)
{
    synapsesOut.push_back(synapse);
}

const double& Neuron::getActivity() const
{
    return activity;
}

const double& Neuron::getBias() const
{
    return bias;
}

const double& Neuron::getOutput() const
{
    return output;
}

Synapse* Neuron::getSynapseFrom(Neuron const * pre) const
{
    for(SynapseList::const_iterator it=synapsesIn.begin(); it!=synapsesIn.end(); it++)
    {
        Synapse * const synapse = *it;
        if (synapse->getPre() == pre) return synapse;
    }
    return NULL;
}

void Neuron::removeSynapseIn(Synapse const * synapse)
{
    synapsesIn.erase(find(synapsesIn.begin(), synapsesIn.end(), synapse));
}

void Neuron::removeSynapseOut(Synapse const * synapse)
{
    synapsesOut.erase(find(synapsesOut.begin(), synapsesOut.end(), synapse));
}

void Neuron::setActivity(const double& aactivity)
{
    activity = aactivity;
}

void Neuron::setBias(const double & abias)
{
    bias = abias;
}

void Neuron::setInput(double const & ainput)
{
    input = ainput;
}

void Neuron::setOutput(const double& aoutput)
{
    output = aoutput;
}

void Neuron::updateActivity()
{
    double newActivity = bias + input;
    for (SynapseList::iterator it = synapsesIn.begin(); it != synapsesIn.end(); it++)
    {
        newActivity += (*it)->getWeight() * (*it)->getPre()->getOutput();
    }
    activity = newActivity;
}

void Neuron::updateOutput()
{
    output = (*func)(activity);
}
