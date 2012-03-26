/*
 * neuron.h
 *
 *  Created on: Feb 28, 2012
 *      Author: timo
 */

#ifndef NEURON_H_
#define NEURON_H_

#include<list>
#include "transferfunction.h"

// forward declarations
class ANN;
class Synapse;
//class TransferFunction;

class Neuron {
public:
    Neuron();
    ~Neuron();
    void addSynapseIn(Synapse * synapse);
    void addSynapseOut(Synapse * synapse);
    const double& getActivity() const;
    const double& getBias() const;
    const double& getOutput() const;
    Synapse* getSynapseFrom(Neuron const * pre) const;
    void removeSynapseIn(Synapse const * synapse);
    void removeSynapseOut(Synapse const * synapse);
    void setActivity(const double& aactivity);
    void setBias(const double& abias);
    void setInput(const double& ainput);
    void setOutput(const double& aoutput);
    void setTransferFunction(TransferFunction const * const afunction);
    void updateActivity();
    void updateOutput();
private:
    typedef std::list<Synapse*> SynapseList;
    double output;
    double activity;
    double bias;
    double input;
    SynapseList synapsesOut;
    SynapseList synapsesIn;
    TransferFunction const * func;
    static TanhFunction const * const tanhFunction;
};

#endif /* NEURON_H_ */
