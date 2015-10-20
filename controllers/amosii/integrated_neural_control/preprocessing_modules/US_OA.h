/*
 * US_OA.h
 *
 *  Created on: Feb 21, 2015
 *      Author: Dennis Goldschmidt
 */

#ifndef US_OA_H_
#define US_OA_H_

#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <ode_robots/amosiisensormotordefinition.h>
#include "utils/ann-framework/ann.h"
using namespace std;

//1) Class for US Obstacle Avoidance Behavior ------------

class US_Obstacleavoidance:  public ANN {

public:
    US_Obstacleavoidance(int mode=3);
    ~US_Obstacleavoidance();

    void step();
    double weight_neuron1;
    double weight_neuron2;
    double weight_neuron3;
    double weight_neuron4;
    double neuron_weightsum_inhib;
    double i1_refl;
    double i2_refl;
    double reflex_cut;
    double e;
    double i1;
    double i2;
    double u1;
    double u2;
    double v1;
    double v2;


    double mu;
    double gamma;
    double vt;

    double mu2;
    double gamma2;
    double vt2;
    double gain;
    int modes;


private:
    bool debug;

};



#endif /* US_OA_H_ */
