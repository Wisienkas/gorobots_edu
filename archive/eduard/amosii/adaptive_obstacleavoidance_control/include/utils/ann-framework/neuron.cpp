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
    while (synapsesIn.size()>0) delete (*(synapsesIn.begin())).second;
    while (synapsesOut.size()>0) delete (*(synapsesOut.begin())).second;
}

void Neuron::addSynapseIn(Synapse * synapse)
{
    synapsesIn.insert( SynapseListPair(synapse->getPre(),synapse) );
}

void Neuron::addSynapseOut(Synapse * synapse)
{
    synapsesOut.insert( SynapseListPair(synapse->getPost(),synapse) );
}

const double& Neuron::getActivity() const
{
    return activity;
}

const double& Neuron::getBias() const
{
    return bias;
}

const double& Neuron::getInput() const
{
    return input;
}

const double& Neuron::getOutput() const
{
    return output;
}

Synapse* Neuron::getSynapseFrom(Neuron const * pre) const
{
    SynapseList::const_iterator it=synapsesIn.find(pre);
    if (it == synapsesIn.end()) return NULL;
    return it->second;
}

Synapse* Neuron::getSynapseTo(Neuron const * post) const
{
    SynapseList::const_iterator it=synapsesOut.find(post);
    if (it == synapsesOut.end()) return NULL;
    return it->second;
}

void Neuron::removeSynapseIn(Synapse const * synapse)
{
    SynapseList::iterator it = synapsesIn.find(synapse->getPre());
    if (it == synapsesIn.end()) return;
    synapsesIn.erase(it);
}

void Neuron::removeSynapseOut(Synapse const * synapse)
{
    SynapseList::iterator it = synapsesOut.find(synapse->getPost());
    if (it == synapsesOut.end()) return;
    synapsesOut.erase(it);
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
        newActivity += it->second->getWeight() * it->first->getOutput();
    }
    activity = newActivity;
}

void Neuron::updateOutput()
{
    output = (*func)(activity);
}
