/*
 * so2cpg.h
 *
 *  Created on: Feb 28, 2012
 *      Author: timo
 */

#ifndef SO2CPG_H_
#define SO2CPG_H_

#include "ann.h"

class SO2CPG : public ANN
{
public:
    SO2CPG();
    void setAlpha(const double aalpha);
    void setPhi(const double& aphi);
protected:
    void updateWeights();
private:
    double phi;
    double alpha;

};


#endif /* SO2CPG_H_ */
