/*
 * vrn.h
 *
 *  Created on: Feb 28, 2012
 *      Author: timo
 */

#ifndef VRN_H_
#define VRN_H_

#include "utils/ann-framework/ann.h"

class VRN : public ANN {
public:
    VRN();
    Neuron* getNeuronX();
    Neuron* getNeuronY();
    Neuron* getNeuronOutput();
private:

};

#endif /* VRN_H_ */
