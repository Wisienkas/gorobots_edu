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
    Neuron* getPre();
    Neuron* getPost();
    const double& getWeight() const;
    void setWeight(const double& aweight);
    //double& getWeightRef();
private:
    Neuron * const pre;
    Neuron * const post;
    double weight;
    //ANN* myANN;
};

#endif /* SYNAPSE_H_ */
