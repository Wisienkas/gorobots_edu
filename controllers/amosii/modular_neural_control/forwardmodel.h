/*
 * forwardmodel.h
 *  Created on: Apr 16, 2012
 *      Author: degoldschmidt
 */

#ifndef FORWARDMODEL_H_
#define FORWARDMODEL_H_

#include <vector>
#include <iostream>
#include <cstdlib>
#include "../../utils/ann-framework/ann.h"
using namespace std;


class ForwardModelANN;


class Forwardmodel {
public:
    Forwardmodel(double rec_weight, double weight, double b, bool learning);
    virtual ~Forwardmodel();
    double step(double motor_output, double sensor_input, bool learning, bool purefoot);
    int counter;
    double rec_w;
    double input_w;
    double lowpass_error_gain;
    double learnrate;
    double error_threshold;
    double error_threshold_2;
    double output;
    double output_old;
    double outputfinal;
    double error;
    double error_old;
    double error_neg;
    double error_elev;
    double lowpass_error;
    double lowpass_error_old;
    double acc_error;
    double acc_error_small;
    double learning_error;
    double bias;
    double activity;
    double input;
    bool finish;



  private:
    ForwardModelANN* FModelANN;
};

#endif /* FORWARDMODEL_H_ */
