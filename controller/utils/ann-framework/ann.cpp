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


#include "ann.h"

#include "neuron.h"
#include "synapse.h"
#include <algorithm>

/** initialization of static const member variables */
TanhFunction const * const ANN::tanhFunctionPointer =
        new TanhFunction();
LogisticFunction const * const ANN::logisticFunctionPointer =
        new LogisticFunction();

ANN::ANN()
{
    setDefaultTransferFunction(tanhFunctionPointer);
}

ANN::~ANN()
{
    setNeuronNumber(0);
    for (AnnList::iterator it=subnets.begin(); it!=subnets.end(); it++)
    {
        delete (*it);
    }
}

Neuron* ANN::addNeuron()
{
    Neuron * const neuron = new Neuron();
    neuron->setTransferFunction(defaultTransferFunction);
    neurons.push_back(neuron);
    return neuron;
}

void ANN::addSubnet(ANN * subnet)
{
    subnets.push_back(subnet);
}

Synapse* ANN::addSynapse(Neuron * post, Neuron * pre)
{
    Synapse * const synapse = new Synapse(post, pre);
    return synapse;
}

void ANN::b(const int neuron, const double& abias)
{
    setBias(neuron, abias);
}

void ANN::b(Neuron* neuron, const double& abias)
{
    setBias(neuron, abias);
}

const double& ANN::b (const int neuron)
{
    return getBias(neuron);
}

const double& ANN::getActivity(const int neuron) const
{
    return getActivity(neurons[neuron]);
}
const double& ANN::getActivity(Neuron const * neuron)
{
    return neuron->getActivity();
}

const double& ANN::getBias(const int neuron) const
{
    return getBias(neurons[neuron]);
}
const double& ANN::getBias(Neuron const * neuron)
{
    return neuron->getBias();
}

TransferFunction const* ANN::getDefaultTransferFunction() const
{
    return defaultTransferFunction;
}

Neuron* ANN::getNeuron(unsigned int const index)
{
    return neurons[index];
}

unsigned int ANN::getNeuronNumber() const
{
    return neurons.size();
}

const double& ANN::getOutput(const int neuron) const
{
    return neurons[neuron]->getOutput();
}

const double& ANN::getOutput(Neuron const * neuron)
{
    return neuron->getOutput();
}

Synapse* ANN::getSynapse(const unsigned int& post, const unsigned int& pre)
        const
{
    return getSynapse(neurons[post], neurons[pre]);
}

Synapse* ANN::getSynapse(Neuron const * const post, Neuron const * const pre)
{
    return post->getSynapseFrom(pre);
}

unsigned int ANN::getTotalNeuronNumber() const
{
    int number = neurons.size();
    for (AnnList::const_iterator it=subnets.begin(); it!=subnets.end(); it++)
    {
        number+=(*it)->getTotalNeuronNumber();
    }
    return number;
}

const double ANN::getWeight(const int& post, const int& pre) const
{
    return getWeight(neurons[post], neurons[pre]);
}

const double ANN::getWeight(Neuron const * post, Neuron const * pre) const
{
    Synapse * synapse = post->getSynapseFrom(pre);
    if (synapse == NULL) return 0.0;
    return synapse->getWeight();

}

Neuron* ANN::n(unsigned int const index)
{
    return getNeuron(index);
}

LogisticFunction const * const ANN::logisticFunction() {
    return logisticFunctionPointer;
}

void ANN::postProcessing()
{
    // nothing to do here
}

void ANN::removeNeuron(Neuron const * neuron)
{
    NeuronList::iterator it = find(neurons.begin(), neurons.end(), neuron);
    if (it != neurons.end()) {
        delete neuron;
        neurons.erase( find(neurons.begin(), neurons.end(), neuron));
    }
    /** @todo give out error message if neuron does not belong to this net */
}

void ANN::setActivity(const int& neuron, const double& aactivity)
{
    setActivity(neurons[neuron], aactivity);
}

void ANN::setActivity(Neuron* neuron, const double & aactivity)
{
    neuron->setActivity(aactivity);
}

void ANN::setAllTransferFunctions(TransferFunction const * const func,
        const bool& includeSubnets)
{
    for (NeuronList::iterator it=neurons.begin(); it != neurons.end(); it++)
    {
        (*it)->setTransferFunction(func);
    }
    if (includeSubnets)
    {
        for (AnnList::iterator it=subnets.begin(); it!=subnets.end(); it++)
        {
            (*it)->setAllTransferFunctions(func, true);
        }
    }
}

void ANN::setBias(Neuron * neuron, const double& abias)
{
    neuron->setBias(abias);
}

void ANN::setBias(const int& neuron, const double& abias)
{
    setBias(neurons[neuron], abias);
}

void ANN::setDefaultTransferFunction(TransferFunction const * const func)
{
    defaultTransferFunction = func;
}

void ANN::setInput(const int& neuron, const double& ainput)
{
    setInput(neurons[neuron], ainput);
}

void ANN::setInput(Neuron* neuron, const double ainput)
{
    neuron->setInput(ainput);
}

void ANN::setOutput(const int& neuron, const double& aoutput)
{
    setOutput(neurons[neuron], aoutput);
}

void ANN::setOutput(Neuron* neuron, const double aoutput)
{
    neuron->setOutput(aoutput);
}

void ANN::setNeuronNumber(const unsigned int& anumber)
{
    while (neurons.size() > anumber)
    {
        removeNeuron(neurons.back());
    }
    while (neurons.size() < anumber)
    {
        addNeuron();
    }
}

TanhFunction const * const ANN::tanhFunction() {
    return tanhFunctionPointer;
}

void ANN::setTransferFunction(const int neuron,
        TransferFunction const * const func)
{
    setTransferFunction(neurons[neuron], func);
}

void ANN::setTransferFunction(Neuron * neuron,
        TransferFunction const * const func)
{
    neuron->setTransferFunction(func);
}


void ANN::setWeight(Neuron* post, Neuron* pre, const double weight)
{
    Synapse * synapse = post->getSynapseFrom(pre);
    if (synapse == NULL) synapse = addSynapse(post, pre);
    synapse->setWeight(weight);
}

void ANN::setWeight(const int post, const int pre, const double weight)
{
    setWeight(neurons[post], neurons[pre], weight);
}

void ANN::step()
{
    updateActivities();
    updateWeights();
    updateOutputs();
    postProcessing();
}

void ANN::updateActivities()
{
    for (NeuronList::iterator it=neurons.begin(); it != neurons.end(); it++)
    {
        (*it)->updateActivity();
    }
    for (AnnList::iterator it=subnets.begin(); it!=subnets.end(); it++)
    {
        (*it)->updateActivities();
    }
}

void ANN::updateOutputs()
{
    for (NeuronList::iterator it=neurons.begin(); it != neurons.end(); it++)
    {
        (*it)->updateOutput();
    }
    for (AnnList::iterator it=subnets.begin(); it!=subnets.end(); it++)
    {
        (*it)->updateOutputs();
    }
}

void ANN::updateWeights()
{
    for (AnnList::iterator it=subnets.begin(); it!=subnets.end(); it++)
    {
        (*it)->updateWeights();
    }
}

void ANN::w(const int& post, const int& pre, const double& aweight)
{
    setWeight(post, pre, aweight);
}

void ANN::w(Neuron* post, Neuron* pre, const double& aweight)
{
    setWeight(post, pre, aweight);
}


const double ANN::w(const int& post, const int& pre)
{
    return getWeight(post, pre);
}
