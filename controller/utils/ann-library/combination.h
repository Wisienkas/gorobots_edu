/*
 * combination.h
 *
 *  Created on: Feb 28, 2012
 *      Author: timo
 */

#ifndef COMBINATION_H_
#define COMBINATION_H_

#include "utils/ann-framework/ann.h"

// forward declarations
class SO2CPG;
class VRN;

/**
 * This is just an example of how to combine several networks into a parent
 * network. DON'T USE this class in your project as it will be removed in the
 * near future
 */
class Combination : public ANN
{
public:
    Combination();
    ~Combination();
    const double& getO1();
    const double& getO2();
private:
    SO2CPG * cpg;
    VRN * vrnLeft;
    VRN * vrnRight;
};


#endif /* COMBINATION_H_ */
