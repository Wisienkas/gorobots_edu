/*
 * ann.cpp
 *
 *  Created on: Feb 28, 2012
 *      Author: timo
 */

#include "ann.h"

#include "neuron.h"
#include "synapse.h"
#include <algorithm>

ANN::ANN()
:tanhFunction()
{
    //parent = NULL;
    setDefaultTransferFunction(&tanhFunction);
}

ANN::~ANN()
{
    setNeuronNumber(0);
    for (AnnList::iterator it=subnets.begin(); it!=subnets.end(); it++)
    {
        delete (*it);
    }
}

//void ANN::addNeuron(Neuron * neuron)
//{
//    neurons.push_back(neuron);
//    //if (parent) parent->addNeuron(neuron);
//}

Neuron* ANN::addNeuron()
{
    Neuron * const neuron = new Neuron();
    neurons.push_back(neuron);//addNeuron(neuron);
    return neuron;
}

void ANN::addSubnet(ANN * subnet)
{
    //subnet->setParent(this);
    subnets.push_back(subnet);
}

Synapse* ANN::addSynapse(Neuron * post, Neuron * pre)
{
    Synapse * const synapse = new Synapse(post, pre);
    //synapses.push_back( synapse );
    return synapse;
}

void ANN::b(const int neuron, const double& abias)
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
const double& ANN::getBias(Neuron const * neuron) const
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

void ANN::removeNeuron(Neuron const * neuron)
{
    delete neuron;
    neurons.erase( find(neurons.begin(), neurons.end(), neuron));
    //if (parent) parent->removeNeuron(neuron);
}

//void ANN::removeSynapse(Synapse const * synapse)
//{
    //synapses.erase( find(synapses.begin(), synapses.end(), synapse));
    //if (parent) parent->removeSynapse(synapse);
//}

void ANN::setBias(Neuron * neuron, const double& abias)
{
    neuron->setBias(abias);
}

void ANN::setBias(const int& neuron, const double& abias)
{
    setBias(neurons[neuron], abias);
}

void ANN::setDefaultTransferFunction(TransferFunction const * func)
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

