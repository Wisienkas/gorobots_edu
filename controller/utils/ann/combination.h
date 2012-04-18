/*
 * combination.h
 *
 *  Created on: Feb 28, 2012
 *      Author: timo
 */

#ifndef COMBINATION_H_
#define COMBINATION_H_

#include "ann.h"

// forward declarations
class SO2CPG;
class VRN;

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
