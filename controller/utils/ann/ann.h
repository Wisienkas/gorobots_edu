/*
 * ann.h
 *
 *  Created on: Feb 28, 2012
 *      Author: timo
 */

#ifndef ANN_H_
#define ANN_H_

#include <list>
#include <vector>
#include "transferfunction.h"

// forward declarations
class Neuron;
class Synapse;

class ANN {
public:
    ANN();
    virtual ~ANN();
    const double&           getActivity(const int neuron) const;
    static const double&    getActivity(Neuron const * neuron);
    const double&           getBias(const int neuron) const;
    const double&           getBias(Neuron const * neuron) const;
    TransferFunction const* getDefaultTransferFunction() const;
    Neuron*                 getNeuron(unsigned int const index);
    unsigned int            getNeuronNumber() const;
    const double&           getOutput(const int neuron) const;
    static const double&    getOutput(Neuron const * neuron);
    unsigned int            getTotalNeuronNumber() const;
    const double            getWeight(const int& post, const int& pre) const;
    const double            getWeight(Neuron const * post, Neuron const * pre) const;
    void                    removeNeuron(Neuron const * neuron);
    void                    setBias(Neuron * neuron, const double& abias);
    void                    setBias(const int& neuron, const double& abias);
    void                    setDefaultTransferFunction(TransferFunction const* func);
    void                    setInput(const int& neuron, const double& ainput);
    static void             setInput(Neuron * neuron, const double ainput);
    void                    setWeight(Neuron* post, Neuron* pre, const double weight);
    void                    setWeight(const int post, const int pre, const double weight);
    virtual void            step();
    virtual void            updateActivities();
    virtual void            updateOutputs();
    virtual void            updateWeights();
protected:
    void            setNeuronNumber(const unsigned int& anumber);
    Neuron*         addNeuron();
    void            addSubnet(ANN* subnet);
    Synapse*        addSynapse(Neuron* post, Neuron* pre);
    void            b(const int neuron, const double& abias);
    const double&   b (const int neuron);
    void            w(const int& post, const int& pre, const double& aweight);
    void            w(Neuron* post, Neuron* pre, const double& aweight);
    const double    w(const int& post, const int& pre);

    const TanhFunction tanhFunction;
private:

    typedef std::vector <Neuron*>   NeuronList;
    typedef std::list <ANN*>        AnnList;
    NeuronList              neurons;
    AnnList                 subnets;
    TransferFunction const* defaultTransferFunction;
};


#endif /* ANN_H_ */
