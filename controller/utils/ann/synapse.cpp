/*
 * synapse.cpp
 *
 *  Created on: Feb 28, 2012
 *      Author: timo
 */

#include "synapse.h"

#include "neuron.h"
#include "ann.h"

Synapse::Synapse(Neuron * const apost, Neuron * const apre)
: pre(apre), post(apost)
{
    pre->addSynapseOut(this);
    post->addSynapseIn(this);
}

Synapse::~Synapse()
{
    //myANN->removeSynapse(this);
    pre->removeSynapseOut(this);
    post->removeSynapseIn(this);
}

Neuron* Synapse::getPost()
{
    return post;
}

Neuron* Synapse::getPre()
{
    return pre;
}

const double& Synapse::getWeight() const
{
    return weight;
}

void Synapse::setWeight(const double & aweight)
{
    weight = aweight;
}
