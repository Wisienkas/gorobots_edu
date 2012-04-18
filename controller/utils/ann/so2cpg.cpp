/*
 * so2cpg.cpp
 *
 *  Created on: Feb 28, 2012
 *      Author: timo
 */

#include "so2cpg.h"

#include <cmath>

SO2CPG::SO2CPG()
{
    setNeuronNumber(2);

    b(0, 0.01);
    b(1, 0.01);

    alpha = 1.01;
    phi   = 0.1*M_PI;
    updateWeights();
}

void SO2CPG::setPhi(const double& aphi)
{
    phi = aphi;
    updateWeights();
}

void SO2CPG::updateWeights()
{
    w(0, 0,  alpha * cos(phi));
    w(0, 1,  alpha * sin(phi));
    w(1, 0, -alpha * sin(phi));
    w(1, 1,  alpha * cos(phi));
}


