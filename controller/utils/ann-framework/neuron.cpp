/*****************************************************************************
 *  Copyright (C) 2012 by Timo Nachstedt                                     *
 *                                                                           *
 *  This program is free software: you can redistribute it and/or modify     *
 *  it under the terms of the GNU General Public License as published by     *
 *  the Free Software Foundation, either version 3 of the License, or        *
 *  (at your option) any later version.                                      *
 *                                                                           *
 *  This program is distributed in the hope that it will be useful,          *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *  GNU General Public License for more details.                             *
 *                                                                           *
 *  You should have received a copy of the GNU General Public License        *
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                           *
 ****************************************************************************/


#include "neuron.h"

#include "ann.h"
#include "synapse.h"
#include "transferfunction.h"
#include <algorithm>
#include <iostream>

// initialize static constant member
TanhFunction const * const Neuron::tanhFunction = new TanhFunction();

Neuron::Neuron()
{
    activity = 0;
    output   = 0;
    bias     = 0;
    input    = 0;
    func     = tanhFunction;
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

void Neuron::setTransferFunction(TransferFunction const * const afunction)
{
    func = afunction;
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
