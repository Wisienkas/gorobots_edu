/*
 * synapse.h
 *
 *  Created on: Feb 28, 2012
 *      Author: timo
 */

#ifndef SYNAPSE_H_
#define SYNAPSE_H_

//forward declarations
class ANN;
class Neuron;

class Synapse {
public:
    Synapse(Neuron * const apost, Neuron * const apre);
    ~Synapse();
    Neuron* getPost();
    Neuron* getPre();
    const double& getWeight() const;
    void setWeight(const double& aweight);
private:
    Neuron * const pre;
    Neuron * const post;
    double weight;
};

#endif /* SYNAPSE_H_ */
